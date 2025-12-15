import os
import json
import random
import math
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

INFO_DESK_NAME = "안내데스크"


def _find_waypoint_yaml(default_name="hospital_waypoints.yaml") -> str:
    """
    우선순위:
    1) ROS param waypoint_file (SmartDispatcher에서 처리)
    2) ENV: HOSPITAL_WAYPOINTS_FILE
    3) ~/.ros/hospital_waypoints.yaml
    4) 현재 파일 기준 상위 경로들 중 config/hospital_waypoints.yaml
    """
    env = os.environ.get("HOSPITAL_WAYPOINTS_FILE")
    if env:
        return env

    cand = os.path.expanduser("~/.ros/hospital_waypoints.yaml")
    if os.path.exists(cand):
        return cand

    here = Path(__file__).resolve()
    for p in [here.parent] + list(here.parents):
        c = p / "config" / default_name
        if c.exists():
            return str(c)

    # 마지막 fallback
    return os.path.expanduser("~/.ros/hospital_waypoints.yaml")


def _yaw_to_quat(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def _coords_to_pose(node: Node, info: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(info["x"])
    pose.pose.position.y = float(info["y"])

    # yaw가 있으면 quaternion으로 변환
    if "yaw" in info:
        _, _, qz, qw = _yaw_to_quat(float(info["yaw"]))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
    else:
        # 기존 호환: w만 있는 케이스(=1.0 등)
        pose.pose.orientation.z = float(info.get("z", 0.0))
        pose.pose.orientation.w = float(info.get("w", 1.0))

    return pose


class SmartDispatcher(Node):
    """
    /hospital/patient_data(JSON) -> departments 중에서 안내데스크는 제외하고 후보 생성
    출발할 때마다 랜덤 대기인원 생성 -> 최소 대기인원 과로 이동
    waypoint 도착 후 /hospital/next_waypoint(True) 오면 다음 출발

    UI publish 제거 버전
    - 좌표는 YAML에서 읽음 (자동 reload)
    """

    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---- waypoint yaml 경로 (param 우선) ----
        self.declare_parameter("waypoint_file", _find_waypoint_yaml())
        self.waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value

        # ---- YAML 캐시 ----
        self._wp_mtime = None
        self._dept_coords = {}  # {name: {x,y,yaw or w}}

        # 최초 로드
        self._reload_waypoints(force=True)

        # ---- 상태 ----
        self.remaining_depts = []
        self.waiting_counts = {}
        self.wait_min = 0
        self.wait_max = 20

        self.current_goal_name = None
        self.current_goal_pose = None
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False

        # ---- home 저장 ----
        self.home_pose = None
        self.home_saved = False
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)

        # ---- Nav2 ----
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # ---- 속도 (초기값 읽기) ----
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # ---- Sub (입력) ----
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)
        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        self.get_logger().info("IDLE: QR 대기 중 (YAML waypoint 적용 버전)")

        # ---- 주기 타이머: Nav2 완료 체크 + yaml 변경 감지 ----
        self.create_timer(0.1, self.loop)

    # =============== YAML 로드/리로드 ===============
    def _reload_waypoints(self, force: bool = False) -> bool:
        path = self.waypoint_file

        try:
            mtime = os.path.getmtime(path) if os.path.exists(path) else None
        except Exception:
            mtime = None

        if (not force) and (mtime is not None) and (self._wp_mtime == mtime):
            return False

        if not os.path.exists(path):
            self.get_logger().warn(f"[waypoints] YAML not found: {path}")
            self._dept_coords = {}
            self._wp_mtime = mtime
            return True

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            depts = data.get("departments", {}) or {}

            # 최소 검증
            cleaned = {}
            for name, info in depts.items():
                if not isinstance(info, dict):
                    continue
                if "x" not in info or "y" not in info:
                    continue
                cleaned[str(name)] = dict(info)

            self._dept_coords = cleaned
            self._wp_mtime = mtime
            self.get_logger().info(f"[waypoints] loaded {len(cleaned)} departments from: {path}")
            return True

        except Exception as e:
            self.get_logger().error(f"[waypoints] YAML load failed: {e}")
            return False

    def _maybe_reload_waypoints(self):
        # loop에서 계속 확인
        self._reload_waypoints(force=False)

    # =============== 콜백들 ===============
    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        if self.home_saved:
            return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose
        self.home_pose = pose
        self.home_saved = True
        self.get_logger().info("[dispatcher] Home pose saved")

    def cb_patient_data(self, msg: String):
        if self.is_emergency:
            self.get_logger().info("EMERGENCY: 복귀 중 (QR 무시)")
            return

        self._reload_waypoints(force=False)

        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
        except Exception as e:
            self.get_logger().error(f"patient_data JSON parse fail: {e}")
            return

        # ✅ 후보 구성(유효한 항목만) + ✅ 안내데스크 제외
        available = set(self._dept_coords.keys())
        self.remaining_depts = [
            d for d in depts
            if (d in available) and (d != INFO_DESK_NAME)
        ]

        if not self.remaining_depts:
            self.get_logger().info("IDLE: 이동할 waypoint 없음 (안내데스크는 후보에서 제외됨)")
            return

        self.get_logger().info("READY: 첫 목적지 출발(최소 대기인원)")
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False

        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        if not msg.data:
            return
        if self.is_emergency:
            return
        if self.waiting_next:
            self.waiting_next = False
            self.get_logger().info("MOVING: 다음 목적지 출발(최소 대기인원)")
            self._start_next_goal()

    def cb_speed(self, msg: Float32):
        self.current_speed = float(self.current_speed) + float(msg.data)
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))
        self._apply_speed(self.current_speed)
        self.get_logger().info(f"[speed] current_speed={self.current_speed:.2f}")

    def cb_pause(self, msg: Bool):
        if msg.data:
            self.is_paused = True
            self.navigator.cancelTask()
            self.get_logger().info("PAUSED")
            return

        # resume
        self.is_paused = False
        if self.is_emergency:
            self.get_logger().info("EMERGENCY: 복귀 중")
            return

        if self.waiting_next:
            self.get_logger().info("ARRIVED: 다음 신호 대기(/hospital/next_waypoint)")
            return

        if self.current_goal_pose is not None:
            self.get_logger().info(f"MOVING(resume): {self.current_goal_name}")
            self.navigator.goToPose(self.current_goal_pose)
        else:
            self.get_logger().info("IDLE")

    def cb_emergency_home(self, msg: Bool):
        if not msg.data:
            return

        self.is_emergency = True
        self.is_paused = False
        self.waiting_next = False

        self.remaining_depts = []
        self.waiting_counts = {}
        self.current_goal_name = None
        self.current_goal_pose = None

        self.navigator.cancelTask()

        if self.home_pose is None:
            self.home_pose = PoseStamped()
            self.home_pose.header.frame_id = "map"
            self.home_pose.pose.position.x = 0.0
            self.home_pose.pose.position.y = 0.0
            self.home_pose.pose.orientation.w = 1.0

        self.get_logger().info("EMERGENCY: HOME 복귀")
        self.navigator.goToPose(self.home_pose)

    # =============== 메인 루프 ===============
    def loop(self):
        # ✅ yaml 변경 감지
        self._maybe_reload_waypoints()

        # emergency/home 복귀 중이면 완료 체크만
        if self.is_emergency:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.get_logger().info("EMERGENCY DONE: HOME 도착")
                else:
                    self.get_logger().info("EMERGENCY DONE: HOME 실패/취소")
                self.is_emergency = False
            return

        if self.is_paused:
            return

        if self.waiting_next:
            return

        if self.current_goal_pose is not None:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"ARRIVED: {self.current_goal_name} (next_waypoint 대기)")
                else:
                    self.get_logger().info(f"FAILED: {self.current_goal_name} (next_waypoint 대기)")
                self.waiting_next = True

    # =============== 내부 유틸 ===============
    def _refresh_waiting_counts(self):
        self.waiting_counts = {
            d: random.randint(self.wait_min, self.wait_max)
            for d in self.remaining_depts
        }

    def _start_next_goal(self):
        if not self.remaining_depts:
            self.current_goal_name = None
            self.current_goal_pose = None
            self.get_logger().info("DONE: 모든 waypoint 완료")
            return

        self._reload_waypoints(force=False)

        self._refresh_waiting_counts()

        min_wait = min(self.waiting_counts.values())
        candidates = [d for d, w in self.waiting_counts.items() if w == min_wait]
        name = random.choice(candidates)

        self.remaining_depts.remove(name)

        info = self._dept_coords.get(name)
        if not info:
            self.get_logger().warn(f"[waypoints] missing coords for: {name} (skip)")
            return

        pose = _coords_to_pose(self, info)

        self.current_goal_name = name
        self.current_goal_pose = pose

        self.get_logger().info(f"MOVING: {name} (wait={self.waiting_counts.get(name)})")
        self.navigator.goToPose(pose)

    def _get_initial_speed_from_velocity_smoother(self) -> float:
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        client.wait_for_service()
        req = GetParameters.Request()
        req.names = ['max_velocity']
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        try:
            arr = fut.result().values[0].double_array_value
            return float(arr[0]) if len(arr) > 0 else 0.25
        except Exception:
            return 0.25

    def _apply_speed(self, speed: float):
        self._set_remote_param('/controller_server', 'FollowPath.max_vel_x', speed)
        self._set_remote_param('/velocity_smoother', 'max_velocity', [speed, 0.0, 1.0])

    def _set_remote_param(self, node_name: str, param_name: str, value):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        client.wait_for_service()

        p = Parameter()
        p.name = param_name

        if isinstance(value, list):
            p.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE_ARRAY,
                double_array_value=[float(x) for x in value]
            )
        else:
            p.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE,
                double_value=float(value)
            )

        req = SetParameters.Request()
        req.parameters = [p]
        client.call_async(req)


def main():
    rclpy.init()
    node = SmartDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
