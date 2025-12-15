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

# 안내데스크 이름 상수
INFO_DESK_NAME = "안내데스크"

# =============================================================================
# Helper Functions (YAML 경로 찾기 및 좌표 변환)
# =============================================================================
def _find_waypoint_yaml(default_name="hospital_waypoints.yaml") -> str:
    """
    YAML 파일 경로 우선순위:
    1) ROS param 'waypoint_file'
    2) ENV 'HOSPITAL_WAYPOINTS_FILE'
    3) ~/.ros/hospital_waypoints.yaml
    4) 현재 파일 상위 config 폴더
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

    if "yaw" in info:
        _, _, qz, qw = _yaw_to_quat(float(info["yaw"]))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
    else:
        pose.pose.orientation.z = float(info.get("z", 0.0))
        pose.pose.orientation.w = float(info.get("w", 1.0))

    return pose


# =============================================================================
# Merged SmartDispatcher Node
# =============================================================================
class SmartDispatcher(Node):
    """
    기능:
    1. /hospital/patient_data 수신 -> 안내데스크 제외하고 방문 리스트 생성
    2. 출발 시 대기인원 랜덤 생성 -> 가장 적은 대기인원 과로 이동
    3. 도착 후 /hospital/next_waypoint(True) 대기 -> 다음 이동
    4. 모든 과 방문 완료 시 -> /hospital/mission_completed 발행 (이메일 트리거) -> Home 복귀
    """

    def __init__(self):
        super().__init__('smart_dispatcher')

        # [설정] Waypoint YAML 로드
        self.declare_parameter("waypoint_file", _find_waypoint_yaml())
        self.waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value

        self._wp_mtime = None
        self._dept_coords = {}  # {name: {x,y,yaw...}}
        self._reload_waypoints(force=True)

        # [상태 변수]
        self.remaining_depts = []
        self.waiting_counts = {}
        self.wait_min = 0
        self.wait_max = 20

        self.current_goal_name = None
        self.current_goal_pose = None
        
        # 플래그들
        self.waiting_next = False  # 도착 후 의사 입력 대기 중인가?
        self.is_paused = False
        self.is_emergency = False
        self.is_returning_home = False # 미션 완료 후 복귀 중인가?

        # [Home 위치 저장] (AMCL Pose 구독)
        self.home_pose = None
        self.home_saved = False
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)

        # [Nav2 Navigator]
        self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active() # 필요시 주석 해제 (시뮬레이션 시작 시간 고려)

        # [속도 제어]
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # -------------------------------
        # Subscribers
        # -------------------------------
        # 1. 미션 데이터 (환자 정보)
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)
        # 2. 다음 단계 진행 신호 (의사 버튼)
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)
        # 3. 유틸리티 (속도, 일시정지, 비상복귀)
        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        # -------------------------------
        # Publishers
        # -------------------------------
        # 미션 완료 시 이메일 전송 트리거
        self.email_pub = self.create_publisher(Bool, '/hospital/mission_completed', 10)

        self.get_logger().info("🟢 SmartDispatcher Started: Ready for QR Data")

        # [메인 루프] 0.1초마다 상태 체크
        self.create_timer(0.1, self.loop)

    # =====================================================
    # YAML 관리
    # =====================================================
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
            return True

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            depts = data.get("departments", {}) or {}

            cleaned = {}
            for name, info in depts.items():
                if isinstance(info, dict) and "x" in info and "y" in info:
                    cleaned[str(name)] = dict(info)

            self._dept_coords = cleaned
            self._wp_mtime = mtime
            self.get_logger().info(f"[waypoints] Loaded {len(cleaned)} locations.")
            return True
        except Exception as e:
            self.get_logger().error(f"[waypoints] YAML load failed: {e}")
            return False

    def _maybe_reload_waypoints(self):
        self._reload_waypoints(force=False)

    # =====================================================
    # Callbacks
    # =====================================================
    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """초기 위치(Home) 자동 저장"""
        if self.home_saved:
            return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose
        self.home_pose = pose
        self.home_saved = True
        self.get_logger().info("🏠 Home pose saved from AMCL")

    def cb_patient_data(self, msg: String):
        """QR 데이터 수신 시 미션 시작"""
        if self.is_emergency:
            self.get_logger().warn("EMERGENCY 상태이므로 새 미션 무시")
            return

        self._reload_waypoints(force=False)

        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
            patient_name = data.get("name", "Unknown")
        except Exception as e:
            self.get_logger().error(f"JSON Parse Error: {e}")
            return

        # 안내데스크 제외 및 유효성 검사
        available = set(self._dept_coords.keys())
        self.remaining_depts = [
            d for d in depts
            if (d in available) and (d != INFO_DESK_NAME)
        ]

        if not self.remaining_depts:
            self.get_logger().warn("갈 수 있는 진료과가 없습니다 (안내데스크 제외됨).")
            return

        self.get_logger().info(f"🚀 미션 시작: 환자 {patient_name}, 방문리스트 {self.remaining_depts}")
        
        # 상태 초기화
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False
        self.is_returning_home = False
        
        # 첫 번째 목적지 출발
        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        """의사 진료 완료 버튼 -> 다음으로 이동"""
        if not msg.data: 
            return
        if self.is_emergency or self.is_returning_home:
            return

        if self.waiting_next:
            self.waiting_next = False
            self.get_logger().info("➡️ 진료 완료 확인. 다음 목적지로 이동합니다.")
            self._start_next_goal()

    def cb_speed(self, msg: Float32):
        self.current_speed = float(self.current_speed) + float(msg.data)
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))
        self._apply_speed(self.current_speed)
        self.get_logger().info(f"[Speed] Adjusted to {self.current_speed:.2f}")

    def cb_pause(self, msg: Bool):
        if msg.data:
            self.is_paused = True
            self.navigator.cancelTask()
            self.get_logger().info("⏸️ PAUSED")
        else:
            self.is_paused = False
            if self.is_emergency or self.is_returning_home:
                self.get_logger().info("▶️ RESUME (Home/Emergency)")
                # Home으로 가던 중이었으면 다시 Home으로
                target = self.home_pose if self.home_pose else self._get_default_home_pose()
                self.navigator.goToPose(target)
                return

            if self.waiting_next:
                self.get_logger().info("⏸️ RESUME: 대기 상태 유지 중")
                return

            if self.current_goal_pose:
                self.get_logger().info(f"▶️ RESUME: {self.current_goal_name}로 이동 재개")
                self.navigator.goToPose(self.current_goal_pose)

    def cb_emergency_home(self, msg: Bool):
        """비상 상황 발생 -> 즉시 집으로"""
        if not msg.data: return

        self.is_emergency = True
        self.is_paused = False
        self.waiting_next = False
        self.is_returning_home = False # Emergency가 우선

        # 데이터 초기화
        self.remaining_depts = []
        self.waiting_counts = {}
        self.current_goal_name = "EMERGENCY_HOME"
        
        self.navigator.cancelTask()
        target = self.home_pose if self.home_pose else self._get_default_home_pose()
        
        self.get_logger().warn("🚨 EMERGENCY: 즉시 HOME 복귀 시작")
        self.navigator.goToPose(target)

    # =====================================================
    # 메인 루프 (Loop)
    # =====================================================
    def loop(self):
        self._maybe_reload_waypoints()

        # 일시정지 상태면 스킵
        if self.is_paused:
            return

        # 대기 상태면 스킵
        if self.waiting_next:
            return
        
        # 이동 중인지 확인
        if not self.navigator.isTaskComplete():
            return  # 아직 이동 중

        # --- 이동 완료(TaskComplete) 후 처리 ---
        
        result = self.navigator.getResult()
        
        # 1. 비상 복귀 완료
        if self.is_emergency:
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("🚨 EMERGENCY 복귀 완료.")
            else:
                self.get_logger().warn("🚨 EMERGENCY 복귀 실패/취소.")
            self.is_emergency = False # 상태 해제 (재시작 가능하게)
            return

        # 2. 미션 완료 후 Home 복귀 완료
        if self.is_returning_home:
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("🏁 모든 일정 종료. HOME 도착 완료. (IDLE)")
            else:
                self.get_logger().warn("🏁 HOME 복귀 실패/취소.")
            self.is_returning_home = False
            return

        # 3. 일반 웨이포인트(진료과) 도착 완료
        if self.current_goal_name:
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"📍 도착: {self.current_goal_name}")
                self.get_logger().info("⏳ 의사 선생님의 입력(/hospital/next_waypoint)을 기다립니다...")
                self.waiting_next = True
            else:
                self.get_logger().error(f"❌ 이동 실패: {self.current_goal_name}. 다음으로 넘어갑니다.")
                # 실패해도 멈추지 않고 다음으로 넘어갈지, 멈출지 결정. 여기선 멈추고 다음 신호 기다림
                self.waiting_next = True
            
            # 도착 처리 후 변수 초기화
            self.current_goal_name = None
            self.current_goal_pose = None

    # =====================================================
    # 로직 (Next Goal & Speed)
    # =====================================================
    def _refresh_waiting_counts(self):
        """남은 과들에 대해 랜덤 대기인원 생성"""
        self.waiting_counts = {
            d: random.randint(self.wait_min, self.wait_max)
            for d in self.remaining_depts
        }

    def _start_next_goal(self):
        """다음 목적지 결정 및 이동 명령"""
        
        # 1. 더 이상 갈 곳이 없으면 -> 완료 처리
        if not self.remaining_depts:
            self.get_logger().info("✅ 모든 진료과 방문 완료!")
            
            # (1) 이메일 발송 트리거
            msg = Bool()
            msg.data = True
            self.email_pub.publish(msg)
            self.get_logger().info("📧 이메일 발송 요청 전송 완료")

            # (2) Home 복귀 시작
            target = self.home_pose if self.home_pose else self._get_default_home_pose()
            self.get_logger().info("🏠 초기 위치(HOME)로 복귀합니다.")
            
            self.is_returning_home = True
            self.navigator.goToPose(target)
            return

        # 2. 갈 곳이 남았으면 -> 스마트 선택
        self._reload_waypoints(force=False)
        self._refresh_waiting_counts()

        # 대기인원이 가장 적은 곳 찾기
        min_wait = min(self.waiting_counts.values())
        candidates = [d for d, w in self.waiting_counts.items() if w == min_wait]
        next_name = random.choice(candidates)

        # 리스트에서 제거
        self.remaining_depts.remove(next_name)

        # 좌표 확인
        info = self._dept_coords.get(next_name)
        if not info:
            self.get_logger().warn(f"좌표 없음: {next_name} (건너뜀)")
            self._start_next_goal() # 재귀 호출로 다음 거 시도
            return

        pose = _coords_to_pose(self, info)

        self.current_goal_name = next_name
        self.current_goal_pose = pose

        self.get_logger().info(f"🧭 이동 시작: {next_name} (대기인원 {self.waiting_counts.get(next_name)}명)")
        self.navigator.goToPose(pose)

    def _get_default_home_pose(self):
        """AMCL 저장된 게 없으면 (0,0) 반환"""
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.orientation.w = 1.0
        return p

    # --- Velocity Smoother Utils ---
    def _get_initial_speed_from_velocity_smoother(self) -> float:
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            return 0.25 # 서비스 없으면 기본값
        
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
        if not client.wait_for_service(timeout_sec=0.5):
            return # 서비스 없으면 무시

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
