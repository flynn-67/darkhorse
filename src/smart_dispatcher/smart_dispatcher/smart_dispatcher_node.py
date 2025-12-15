import json
import time
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
    "영상의학과":    {"x": 6.578537940979004,  "y": 2.621462106704712,  "w": 1.0},
    "내과":          {"x": 7.445363998413086,  "y": 0.5102964639663696, "w": 1.0},
    "정형외과":      {"x": 0.753912627696991,  "y": -2.640972375869751, "w": 1.0},
    "신경과":        {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
}


class SmartDispatcher(Node):
    """
    QR 완료(/hospital/patient_data) → 후보 과 리스트 생성 → (출발할 때마다) 랜덤 대기인원 생성
    → 대기인원 "최솟값" 과로 Nav2 주행
    각 waypoint 도착 시 멈춤 → /hospital/next_waypoint 신호(True) 오면 다음 출발(그때 대기인원 다시 랜덤)
    환자용 UI는 속도/정지/긴급복귀만 제어
    """
    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---- 상태 ----
        self.remaining_depts = []        # 아직 방문 안 한 과(후보)
        self.waiting_counts = {}         # {과: 대기인원}
        self.wait_min = 0               # 랜덤 대기인원 최소
        self.wait_max = 20              # 랜덤 대기인원 최대

        self.current_goal_name = None
        self.current_goal_pose = None
        self.waiting_next = False        # waypoint 도착 후 다음 신호 대기
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
        self.step = 0.05

        # ---- Sub (입력) ----
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)
        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        # ---- Pub (UI 표시) ----
        self.pub_status  = self.create_publisher(String,  '/nav_status', 10)
        self.pub_target  = self.create_publisher(String,  '/nav_current_target', 10)
        self.pub_speed   = self.create_publisher(Float32, '/nav_current_speed', 10)

        # (선택) UI/디버그용: 현재 대기인원 맵 JSON publish
        self.pub_waiting = self.create_publisher(String, '/hospital/waiting_counts', 10)

        self._publish_status("IDLE: QR 대기 중")
        self._publish_speed()

        # ---- 주기 타이머: Nav2 완료 체크 ----
        self.create_timer(0.1, self.loop)

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
        """
        QR 완료 후 patient_data(JSON) 수신하면 후보 리스트 세팅하고 첫 출발
        """
        if self.is_emergency:
            self._publish_status("EMERGENCY: 복귀 중 (QR 무시)")
            return

        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
        except Exception as e:
            self.get_logger().error(f"patient_data JSON parse fail: {e}")
            return

        # 후보 구성(유효한 항목만)
        self.remaining_depts = [d for d in depts if d in DEPARTMENT_COORDINATES]
        if not self.remaining_depts:
            self._publish_status("IDLE: 이동할 waypoint 없음")
            return

        self._publish_status("READY: 첫 목적지 출발(최소 대기인원)")
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False

        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        """
        도착 후 대기 상태일 때만 다음 출발
        """
        if not msg.data:
            return
        if self.is_emergency:
            return
        if self.waiting_next:
            self.waiting_next = False
            self._publish_status("MOVING: 다음 목적지 출발(최소 대기인원)")
            self._start_next_goal()

    def cb_speed(self, msg: Float32):
        self.current_speed = float(self.current_speed) + float(msg.data)
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))
        self._apply_speed(self.current_speed)
        self._publish_speed()

    def cb_pause(self, msg: Bool):
        """
        True: 정지(현재 task cancel)
        False: 재개(현재 목표로 다시 goToPose)
        """
        if msg.data:
            self.is_paused = True
            self.navigator.cancelTask()
            self._publish_status("PAUSED")
            return

        # resume
        self.is_paused = False
        if self.is_emergency:
            self._publish_status("EMERGENCY: 복귀 중")
            return

        # 도착 대기 상태면 재개할 게 없음
        if self.waiting_next:
            self._publish_status("ARRIVED: 다음 신호 대기(Enter)")
            return

        if self.current_goal_pose is not None:
            self._publish_status(f"MOVING: {self.current_goal_name}")
            self.navigator.goToPose(self.current_goal_pose)
        else:
            self._publish_status("IDLE")

    def cb_emergency_home(self, msg: Bool):
        """
        True면 즉시 멈추고 Home으로 복귀 (후보/목표 초기화)
        """
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

        self._publish_status("EMERGENCY: HOME 복귀")
        self.navigator.goToPose(self.home_pose)

    # =============== 메인 루프 ===============
    def loop(self):
        # emergency/home 복귀 중이면 완료 체크만
        if self.is_emergency:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self._publish_status("EMERGENCY DONE: HOME 도착")
                else:
                    self._publish_status("EMERGENCY DONE: HOME 실패/취소")
                self.is_emergency = False
            return

        # pause면 아무 것도 하지 않음
        if self.is_paused:
            return

        # 도착 후 다음 대기 상태면 아무 것도 하지 않음
        if self.waiting_next:
            return

        # 현재 주행 중이면 완료 체크
        if self.current_goal_pose is not None:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self._publish_status(f"ARRIVED: {self.current_goal_name} (Enter로 다음)")
                    self.waiting_next = True
                else:
                    self._publish_status(f"FAILED: {self.current_goal_name} (Enter로 다음)")
                    self.waiting_next = True

    # =============== 내부 유틸 ===============
    def _refresh_waiting_counts(self):
        # 출발할 때마다: 현재 남아있는 후보 과들에 대해 랜덤 대기인원 갱신
        self.waiting_counts = {
            d: random.randint(self.wait_min, self.wait_max)
            for d in self.remaining_depts
        }

        # (선택) UI/디버그용으로 publish
        payload = {"waiting": self.waiting_counts}
        self.pub_waiting.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _start_next_goal(self):
        if not self.remaining_depts:
            self.current_goal_name = None
            self.current_goal_pose = None
            self._publish_status("DONE: 모든 waypoint 완료")
            return

        # ✅ 출발할 때마다 대기인원 랜덤 생성(갱신)
        self._refresh_waiting_counts()

        # ✅ 대기인원 "최솟값"인 과 선택
        # 동점이면(같은 최소값 여러 개면) 그중 하나 랜덤 선택
        min_wait = min(self.waiting_counts.values())
        candidates = [d for d, w in self.waiting_counts.items() if w == min_wait]
        name = random.choice(candidates)

        # 방문 처리(다음 선택에서 제외)
        self.remaining_depts.remove(name)

        info = DEPARTMENT_COORDINATES[name]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(info["x"])
        pose.pose.position.y = float(info["y"])
        pose.pose.orientation.w = float(info.get("w", 1.0))

        self.current_goal_name = name
        self.current_goal_pose = pose

        self.pub_target.publish(String(data=name))
        self._publish_status(f"MOVING: {name} (wait={self.waiting_counts.get(name)})")
        self.navigator.goToPose(pose)

    def _publish_status(self, s: str):
        self.pub_status.publish(String(data=s))
        self.get_logger().info(s)

    def _publish_speed(self):
        self.pub_speed.publish(Float32(data=float(self.current_speed)))

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
        # controller_server (DWB)
        self._set_remote_param('/controller_server', 'FollowPath.max_vel_x', speed)
        # velocity_smoother (최종 상한)
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
