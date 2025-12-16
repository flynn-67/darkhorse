import math
import json
import random
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics

# ROS 2 Messages / Actions
from limo_interfaces.action import Speak as speakActionMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry


# ==========================================
# 상수 및 좌표 정의
# ==========================================
INFO_DESK_NAME = "안내데스크"

# 좌표: (0.08, 0.08)은 출발지(0,0) 근처입니다. 
# 로봇이 벽 속에 있지 않다면 이동 가능한 좌표입니다.
DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": -2.0478696823120117, "y": 1.3148077726364136, "w": 1.0},
    "정형외과":      {"x": 4.325248718261719, "y": -1.067739486694336, "w": 1.0},
    "안내데스크":    {"x": 0.08828259259462357, "y": 0.08828259259462357, "w": 1.0},
}
DEFAULT_DEPARTMENTS = ["진단검사의학과", "정형외과"]

# UI 상태 보고 헬퍼
def publish_ui_status(ros_node, text):
    pub = ros_node.create_publisher(String, '/hospital/nav_status', 10)
    msg = String()
    msg.data = text
    pub.publish(msg)


# ==========================================
# Action Nodes
# ==========================================
class GoToInfoDesk(ActionWithROSAction):
    """안내데스크로 이동하는 Nav2 액션 노드 (타임아웃 및 강제 성공 기능 포함)"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
        # ✅ [필수 추가] 타임아웃 60초 설정
        self.timeout_sec = 60.0
        self.start_time = None
        self.nav_goal_sent = False

    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))

        # UI 업데이트
        publish_ui_status(self.ros.node, "안내데스크 복귀 중 🏠")
        print("[GoToInfoDesk] 🏠 안내데스크로 복귀 시작")
        
        # ✅ [필수 추가] 시작 시간 기록
        self.start_time = self.ros.node.get_clock().now()
        self.nav_goal_sent = True
        return goal

    # ✅ [핵심 기능] 이 함수가 없어서 멈췄던 것입니다!
    # 60초 동안 도착 못하면 강제로 성공 처리하고 사이렌을 끄러 갑니다.
    async def run(self, agent, bb):
        # 1. 부모 로직 실행
        status = await super().run(agent, bb)

        # 2. 이동 중(RUNNING)이라면 시간 체크
        if status == Status.RUNNING and self.nav_goal_sent:
            now = self.ros.node.get_clock().now()
            elapsed_time = (now - self.start_time).nanoseconds / 1e9
            
            # 60초 타임아웃 발생 시
            if elapsed_time > self.timeout_sec:
                print(f"[GoToInfoDesk] ⚠️ 60초 타임아웃! 이동 포기하고 사이렌 끕니다.")
                
                # Nav2 취소 요청
                if self._action_client and self._goal_handle:
                    self._action_client.cancel_goal_async(self._goal_handle)
                
                self.nav_goal_sent = False
                # 비상 상황 종료를 위해 SUCCESS 반환 -> ControlSiren(false) 실행됨
                return Status.SUCCESS
            
        return status

    # ✅ [핵심 기능] 이동 실패해도 비상 상황이면 SUCCESS 반환
    def _interpret_result(self, result, agent, bb, status_code=None):
        self.nav_goal_sent = False # 종료 플래그

        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[GoToInfoDesk] ✅ 안내데스크 도착 (SUCCESS)")
            return Status.SUCCESS
        
        # 만약 비상 상황이었다면 (abort=True)
        if bb.get('abort', False):
            print(f"[GoToInfoDesk] ⚠️ 이동 실패했으나 비상 상황 종료를 위해 SUCCESS 반환.")
            publish_ui_status(self.ros.node, "복귀 완료 (강제)")
            return Status.SUCCESS
            
        print(f"[GoToInfoDesk] ❌ 안내데스크 복귀 실패/취소 (Code: {status_code})")
        return Status.FAILURE


class WaitForQR(SyncAction):
    """QR 코드 수신 대기 노드 (초기화 기능 포함)"""
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        self.done = False
        self.sub = agent.ros_bridge.node.create_subscription(String, "/hospital/qr_login", self._callback, 10)
        self.home_saved = False
        self.first_run = True

    def _callback(self, msg):
        self.received_msg = msg
        print(f"[WaitForQR] 📨 QR 데이터 수신: {msg.data}")

    def _tick(self, agent, bb):
        # ✅ 초기화 로직 (정상)
        if self.first_run:
            publish_ui_status(agent.ros_bridge.node, "환자 접수 대기 중... 📋")
            bb['abort'] = False  # 비상 상태 초기화
            self.first_run = False

        if self.done: return Status.SUCCESS
        
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                bb['home_pose'] = agent.robot_pose
                self.home_saved = True

        if self.received_msg is None: return Status.RUNNING

        try:
            data = json.loads(self.received_msg.data)
            bb['patient_id'] = data.get("patient_id", "Unknown")
            raw_depts = data.get("departments", DEFAULT_DEPARTMENTS)
            depts = [d for d in raw_depts if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)]

            bb['department_queue'] = list(depts)
            bb['remaining_depts'] = list(depts)
            bb['speak_text'] = "접수가 완료되었습니다. 이동을 시작할게요."

            self.received_msg = None
            self.done = True
            publish_ui_status(agent.ros_bridge.node, f"환자 {bb['patient_id']} 접수 완료 ✅")
            return Status.SUCCESS

        except Exception as e:
            print(f"[WaitForQR] ⚠️ JSON 파싱 에러: {e}")
            self.received_msg = None
            return Status.RUNNING


class Think(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.wait_min = 0; self.wait_max = 20

    def _tick(self, agent, bb):
        remaining = bb.get('remaining_depts', []) or []
        if INFO_DESK_NAME in remaining: remaining = [d for d in remaining if d != INFO_DESK_NAME]
        if len(remaining) == 0: return Status.FAILURE

        waiting_counts = {d: random.randint(self.wait_min, self.wait_max) for d in remaining}
        min_wait = min(waiting_counts.values())
        candidates = [d for d, w in waiting_counts.items() if w == min_wait]
        next_dept = random.choice(candidates)

        coords = DEPARTMENT_COORDINATES.get(next_dept)
        if not coords:
            remaining.remove(next_dept)
            bb['remaining_depts'] = remaining
            return Status.RUNNING

        bb['current_target_name'] = next_dept
        bb['current_target_coords'] = coords
        remaining.remove(next_dept)
        bb['remaining_depts'] = remaining
        bb['speak_text'] = f"{next_dept}로 이동할게요. 대기인원 {waiting_counts[next_dept]}명."
        return Status.SUCCESS


class Move(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        target_name = bb.get('current_target_name', '알 수 없음')
        if not coords: return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))

        publish_ui_status(self.ros.node, f"{target_name} 이동 중 🚑")
        print(f"[Move] 🚀 {target_name}로 이동 시작")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        target_name = bb.get('current_target_name', '목적지')
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['speak_text'] = f"{target_name}에 도착했습니다."
            return Status.SUCCESS

        bb['speak_text'] = f"{target_name}로 이동하지 못했습니다."
        return Status.FAILURE


class WaitDoctorDone(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self._done = False
        self.sub = agent.ros_bridge.node.create_subscription(Bool, "/hospital/doctor_input", self._cb, 10)
        self.status_sent = False

    def _cb(self, msg: Bool): 
        if msg.data is True: 
            print("[WaitDoctorDone] 👨‍⚕️ 의사 입력 수신됨!")
            self._done = True

    def _tick(self, agent, bb):
        if not self.status_sent:
            target_name = bb.get('current_target_name', '진료과')
            publish_ui_status(agent.ros_bridge.node, f"{target_name} 진료 중... 👨‍⚕️")
            self.status_sent = True

        if not self._done: return Status.RUNNING

        self._done = False; self.status_sent = False
        bb['speak_text'] = "진료 종료. 다음으로 이동."
        return Status.SUCCESS


class ReturnHome(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))
        return goal


class SpeakAction(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (speakActionMsg, 'speak_text'))
    def _build_goal(self, agent, bb):
        text = bb.pop('speak_text', None)
        if not text: return None
        goal = speakActionMsg.Goal()
        goal.text = text
        return goal


class WaitSpeedOK(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.limit = 0.8
        self._odom = None
        self._warned = False
        self.sub = agent.ros_bridge.node.create_subscription(Odometry, "/odom", self._cb, 10)

    def _cb(self, msg: Odometry): self._odom = msg

    def _tick(self, agent, bb):
        if self._odom is None: return Status.SUCCESS
        if abs(self._odom.twist.twist.linear.x) > self.limit:
            if not self._warned:
                bb['speak_text'] = f"속도 위반! {self.limit} 이하로 줄이세요."
                self._warned = True
            return Status.SUCCESS
        self._warned = False
        return Status.SUCCESS


# ==========================================
# Condition Nodes
# ==========================================
class IsEmergencyPressed(ConditionWithROSTopics):
    """비상 버튼 감지 노드 (Latch 기능 포함)"""
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, agent, [(Bool, "/emergency_trigger", "emergency_flag")], **kwargs)

    async def run(self, agent, bb):
        # 1. 이미 비상 복귀 중(abort=True)이라면 무조건 SUCCESS
        if bb.get('abort', False):
            return Status.SUCCESS

        # 2. 버튼 상태 확인
        if "emergency_flag" not in self._cache: return Status.FAILURE
        is_pressed = self._cache["emergency_flag"].data
        return Status.SUCCESS if is_pressed else Status.FAILURE


class IsBatteryLow(ConditionWithROSTopics):
    def __init__(self, name, agent): super().__init__(name, agent, [(Bool, "/battery_low", "battery_flag")])
    def _predicate(self, agent, bb):
        if "battery_flag" in self._cache and self._cache["battery_flag"].data: return True
        return False


# ==========================================
# Abort / Siren / Email
# ==========================================
class SetAbort(SyncAction):
    def __init__(self, name, agent): super().__init__(name, self._tick)
    def _tick(self, agent, bb):
        bb['abort'] = True
        bb['speak_text'] = "비상 상황 발생! 복귀합니다."
        print("[Abort] 🚨 비상 플래그 설정")
        return Status.SUCCESS


# ✅ [NotAbort 유지] XML과 이름 일치시킴
class NotAbort(SyncAction):
    def __init__(self, name, agent): super().__init__(name, self._tick)
    def _tick(self, agent, bb):
        return Status.FAILURE if bb.get('abort', False) else Status.SUCCESS


class SendDiagnosisEmail(SyncAction):
    def __init__(self, name, agent, topic="/hospital/send_diagnosis_email", **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(String, topic, 10)
    def _tick(self, agent, bb):
        payload = {"patient_id": bb.get("patient_id", "Unknown"), "email": bb.get("patient_email") or bb.get("email"), "request": "send_diagnosis_email"}
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        return Status.SUCCESS


class ControlSiren(SyncAction):
    def __init__(self, name, agent, enable=True, **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(Bool, "/cmd_siren", 10)
        self.enable_siren = bool(enable)
        if 'enable' in kwargs:
            val = str(kwargs['enable']).lower()
            self.enable_siren = (val == 'true')
    def _tick(self, agent, bb):
        msg = Bool(); msg.data = self.enable_siren
        self.pub.publish(msg)
        publish_ui_status(self.ros.node, f"🚨 사이렌 {'ON' if self.enable_siren else 'OFF'}")
        print(f"[Siren] 📢 사이렌 제어: {self.enable_siren}")
        return Status.SUCCESS


# ==========================================
# Control Nodes
# ==========================================
class KeepRunningUntilFailure(Node):
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []
    async def run(self, agent, bb):
        if not self.children: return Status.FAILURE
        status = await self.children[0].run(agent, bb)
        if status == Status.FAILURE: return Status.FAILURE
        return Status.RUNNING


# ==========================================
# BT 노드 등록
# ==========================================
CUSTOM_ACTION_NODES = [
    'WaitForQR', 'SpeakAction', 'Think', 'WaitSpeedOK', 'Move',
    'WaitDoctorDone', 'ReturnHome', 'GoToInfoDesk', 'SendDiagnosisEmail',
    'SetAbort', 'NotAbort', 'ControlSiren',
]
CUSTOM_CONDITION_NODES = ['IsEmergencyPressed', 'IsBatteryLow']

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')

print(f"✅ 커스텀 노드 등록 완료: action={len(CUSTOM_ACTION_NODES)}, condition={len(CUSTOM_CONDITION_NODES)}")
