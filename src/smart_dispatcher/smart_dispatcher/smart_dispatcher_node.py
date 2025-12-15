import json
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
    "안내데스크":    {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
}
INFO_DESK_NAME = "안내데스크"


class SmartDispatcher(Node):
    """
    /hospital/patient_data(JSON) -> departments 중에서 안내데스크는 제외하고 후보 생성
    출발할 때마다 랜덤 대기인원 생성 -> 최소 대기인원 과로 이동
    waypoint 도착 후 /hospital/next_waypoint(True) 오면 다음 출발

    + 도착 성공 시 /hospital/arrival_status(String)에 현재 과 이름 publish (doctor_ui_trigger 등이 사용)
    """

    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---- 상태 ----
        self.remaining_depts = []        # 아직 방문 안 한 과(후보) (안내데스크 제외)
        self.waiting_counts = {}         # {과: 대기인원}
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
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10
        )

        # ---- Nav2 ----
        self.navigator = BasicNavigator()
        # Nav2가 올라오지 않았으면 여기서 대기함 (필요하면 시뮬 상황에 맞게 조절)
        try:
            self.navigator.waitUntilNav2Active()
        except Exception as e:
            self.get_logger().warn(f"waitUntilNav2Active() 예외: {e}")

        # ---- 속도 (초기값 읽기) ----
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # ---- 도착 알림용 토픽 ----
        self.pub_arrival_status = self.create_publisher(String, '/hospital/arrival_status', 10)

        # ---- Sub (입력) ----
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)
        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        self.get_logger().info("IDLE: QR 대기 중 (dispatcher ready)")

        # ---- 주기 타이머: Nav2 완료 체크 ----
        self.create_timer(0.1, self.loop)

    # ---------------- 콜백 ----------------
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
        안내데스크는 무조건 제외됨
        """
        if self.is_emergency:
            self.get_logger().info("EMERGENCY: 복귀 중 (QR 무시)")
            return

        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
        except Exception as e:
            self.get_logger().error(f"patient_data JSON parse fail: {e}")
            return

        self.remaining_depts = [
            d for d in depts
            if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)
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
        """
        도착 후 대기 상태일 때만 다음 출발
        """
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
        """
        True: 정지(현재 task cancel)
        False: 재개(현재 목표로 다시 goToPose)
        """
        if msg.data:
            self.is_paused = True
            self.navigator.cancelTask()
            self.get_logger().info("PAUSED")
            return

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

        self.get_logger().info("EMERGENCY: HOME 복귀")
        self.navigator.goToPose(self.home_pose)

    # ---------------- 메인 루프 ----------------
    def loop(self):
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

        if self.is_paused or self.waiting_next:
            return

        if self.current_goal_pose is not None and self.navigator.isTaskComplete():
            res = self.navigator.getResult()

            if res == TaskResult.SUCCEEDED:
                self.get_logger().info(f"ARRIVED: {self.current_goal_name} (next_waypoint 대기)")

                # ✅ 도착 성공 방송
                m = String()
                m.data = self.current_goal_name
                self.pub_arrival_status.publish(m)
            else:
                self.get_logger().info(f"FAILED: {self.current_goal_name} (next_waypoint 대기)")

            self.waiting_next = True

    # ---------------- 내부 유틸 ----------------
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

        self._refresh_waiting_counts()

        min_wait = min(self.waiting_counts.values())
        candidates = [d for d, w in self.waiting_counts.items() if w == min_wait]
        name = random.choice(candidates)

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

        self.get_logger().info(f"MOVING: {name} (wait={self.waiting_counts.get(name)})")
        self.navigator.goToPose(pose)

    def _get_initial_speed_from_velocity_smoother(self) -> float:
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("velocity_smoother/get_parameters 서비스 없음 -> 기본 0.25 사용")
            return 0.25

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
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{node_name}/set_parameters 서비스 없음 -> {param_name} 설정 스킵")
            return

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
