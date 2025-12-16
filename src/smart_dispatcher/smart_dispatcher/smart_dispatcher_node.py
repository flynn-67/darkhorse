import json
import random
import time

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
    /hospital/patient_data(JSON) 수신 -> 안내데스크 제외한 과 리스트 생성 -> 최소 대기 인원 과로 이동
    목적지 도착 -> /hospital/arrival_status(String) publish
    의사 진료 완료 -> doctor_app.py가
      - 다음 waypoint 있으면: /hospital/next_waypoint(True)
      - 다음 waypoint 없으면: /hospital/return_home(True)
    + doctor_app.py가 판단할 수 있도록 /hospital/has_next_waypoint(Bool) publish
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
        self.is_returning_home = False   # ✅ return_home 진행 중인지

        # ---- home 저장 ----
        self.home_pose = None
        self.home_saved = False
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10
        )

        # ---- Nav2 ----
        self.navigator = BasicNavigator()
        try:
            self.navigator.waitUntilNav2Active()
        except Exception as e:
            self.get_logger().warn(f"waitUntilNav2Active() 예외: {e}")

        # ---- 속도 ----
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # ---- Pub ----
        self.pub_arrival_status = self.create_publisher(String, '/hospital/arrival_status', 10)

        # ✅ doctor_app.py가 읽는 토픽
        self.pub_has_next = self.create_publisher(Bool, '/hospital/has_next_waypoint', 10)

        # ---- Sub ----
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)

        # 기존 next_waypoint 신호 유지
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)

        # ✅ doctor_app.py가 보내는 복귀 신호
        self.create_subscription(Bool,    '/hospital/return_home',    self.cb_return_home, 10)

        # ✅ (권장) 진료 완료 신호: 이 신호만으로도 다음/복귀 판단 가능하게 호환 추가
        self.create_subscription(Bool,    '/hospital/doctor_input',   self.cb_doctor_input, 10)

        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        self.get_logger().info("IDLE: QR 대기 중 (dispatcher ready)")

        # ---- 주기 타이머 ----
        self.last_has_next_pub = 0.0
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
            self.get_logger().info("IDLE: 이동할 waypoint 없음 (안내데스크 제외)")
            self._publish_has_next(False)
            return

        self.get_logger().info("READY: 첫 목적지 출발(최소 대기인원)")
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False
        self.is_returning_home = False

        # 첫 출발 직전: 남은 waypoint 있음
        self._publish_has_next(True)

        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        if not msg.data:
            return
        if self.is_emergency or self.is_returning_home:
            return
        if self.waiting_next:
            # 혹시 남은 waypoint가 없는데 next가 들어오면 -> 안내데스크 복귀
            if not self.remaining_depts:
                self.get_logger().info("DONE: 다음 waypoint 없음 -> 안내데스크 복귀")
                self.waiting_next = False
                self._go_to_info_desk(clear_state=True)
                return

            self.waiting_next = False
            self.get_logger().info("MOVING: 다음 목적지 출발(최소 대기인원)")
            self._start_next_goal()

    def cb_return_home(self, msg: Bool):
        """doctor_app.py에서 마지막이면 이 토픽을 True로 보냄"""
        if not msg.data:
            return
        if self.is_emergency:
            return

        self.get_logger().info("RETURN_HOME: 안내데스크 복귀 요청 수신")
        self.waiting_next = False
        self.is_paused = False
        self.navigator.cancelTask()
        self._go_to_info_desk(clear_state=True)

    def cb_doctor_input(self, msg: Bool):
        """
        doctor_app.py는 항상 doctor_input(True)를 보냄.
        (UI가 next/return도 같이 보내지만, 이 토픽 하나만으로도 동작하도록 안전하게 호환)
        """
        if not msg.data:
            return
        if self.is_emergency or self.is_returning_home:
            return
        if not self.waiting_next:
            return  # 도착해서 대기중일 때만 처리

        if self.remaining_depts:
            self.get_logger().info("DOCTOR_DONE: 다음 waypoint 존재 -> 출발")
            self.waiting_next = False
            self._start_next_goal()
        else:
            self.get_logger().info("DOCTOR_DONE: 다음 waypoint 없음 -> 안내데스크 복귀")
            self.waiting_next = False
            self._go_to_info_desk(clear_state=True)

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

        self.is_paused = False

        if self.is_emergency:
            self.get_logger().info("EMERGENCY: 복귀 중")
            return
        if self.is_returning_home:
            self.get_logger().info("RETURN_HOME: 복귀 중")
            return
        if self.waiting_next:
            self.get_logger().info("ARRIVED: 다음 신호 대기")
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
        self.is_returning_home = False

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
        # ✅ 주기적으로 has_next_waypoint 방송 (UI가 안정적으로 받게)
        now = time.time()
        if now - self.last_has_next_pub > 0.3:
            self._publish_has_next(len(self.remaining_depts) > 0)
            self.last_has_next_pub = now

        # emergency 복귀 완료 체크
        if self.is_emergency:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.get_logger().info("EMERGENCY DONE: HOME 도착")
                else:
                    self.get_logger().info("EMERGENCY DONE: HOME 실패/취소")
                self.is_emergency = False
            return

        # return_home(안내데스크) 복귀 완료 체크
        if self.is_returning_home:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.get_logger().info("RETURN_HOME DONE: 안내데스크 도착 -> IDLE")
                else:
                    self.get_logger().info("RETURN_HOME DONE: 안내데스크 실패/취소")
                self.is_returning_home = False
                self.current_goal_name = None
                self.current_goal_pose = None
            return

        if self.is_paused or self.waiting_next:
            return

        if self.current_goal_pose is not None and self.navigator.isTaskComplete():
            res = self.navigator.getResult()

            if res == TaskResult.SUCCEEDED:
                self.get_logger().info(f"ARRIVED: {self.current_goal_name} (doctor input 대기)")
                m = String()
                m.data = self.current_goal_name
                self.pub_arrival_status.publish(m)
            else:
                self.get_logger().info(f"FAILED: {self.current_goal_name} (doctor input 대기)")

            # ✅ 도착했으니 UI가 판단할 수 있게 has_next 즉시 갱신
            self._publish_has_next(len(self.remaining_depts) > 0)

            self.waiting_next = True

    # ---------------- 내부 유틸 ----------------
    def _publish_has_next(self, flag: bool):
        msg = Bool()
        msg.data = bool(flag)
        self.pub_has_next.publish(msg)

    def _refresh_waiting_counts(self):
        self.waiting_counts = {
            d: random.randint(self.wait_min, self.wait_max)
            for d in self.remaining_depts
        }

    def _start_next_goal(self):
        if not self.remaining_depts:
            self.current_goal_name = None
            self.current_goal_pose = None
            self.get_logger().info("DONE: 모든 waypoint 완료 (UI가 return_home 보내면 안내데스크로 복귀)")
            self._publish_has_next(False)
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

        # ✅ 다음 목적지로 출발했으니 남은 waypoint 갱신
        self._publish_has_next(len(self.remaining_depts) > 0)

        self.get_logger().info(f"MOVING: {name} (wait={self.waiting_counts.get(name)})")
        self.navigator.goToPose(pose)

    def _go_to_info_desk(self, clear_state: bool = False):
        """안내데스크 좌표로 복귀"""
        info = DEPARTMENT_COORDINATES[INFO_DESK_NAME]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(info["x"])
        pose.pose.position.y = float(info["y"])
        pose.pose.orientation.w = float(info.get("w", 1.0))

        if clear_state:
            self.remaining_depts = []
            self.waiting_counts = {}
            self._publish_has_next(False)

        self.is_returning_home = True
        self.current_goal_name = INFO_DESK_NAME
        self.current_goal_pose = pose
        self.get_logger().info("MOVING: 안내데스크 복귀")
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
