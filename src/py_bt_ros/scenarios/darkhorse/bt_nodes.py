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

DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
    "영상의학과":    {"x": 6.578537940979004,  "y": 2.621462106704712,  "w": 1.0},
    "내과":          {"x": 7.445363998413086,  "y": 0.5102964639663696, "w": 1.0},
    "정형외과":      {"x": 0.753912627696991,  "y": -2.640972375869751, "w": 1.0},
    "안내데스크":    {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
}
DEFAULT_DEPARTMENTS = ["진단검사의학과", "영상의학과", "내과", "정형외과"]


# ==========================================
# Action Nodes
# ==========================================
class GoToInfoDesk(ActionWithROSAction):
    """안내데스크로 이동하는 Nav2 액션 노드"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords:
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))

        print("[GoToInfoDesk] 🏠 안내데스크로 복귀 시작")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[GoToInfoDesk] ✅ 안내데스크 도착 (SUCCESS)")
            return Status.SUCCESS
        print(f"[GoToInfoDesk] ❌ 안내데스크 복귀 실패/취소 (Code: {status_code})")
        return Status.FAILURE


class WaitForQR(SyncAction):
    """QR 코드(/hospital/qr_login) 수신 대기 노드"""
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        self.done = False

        self.sub = agent.ros_bridge.node.create_subscription(
            String, "/hospital/qr_login", self._callback, 10
        )

        self.home_saved = False

    def _callback(self, msg):
        self.received_msg = msg
        print(f"[WaitForQR] 📨 QR 데이터 수신: {msg.data}")

    def _tick(self, agent, bb):
        if self.done:
            return Status.SUCCESS

        # 로봇의 초기 위치 저장 (선택)
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                bb['home_pose'] = agent.robot_pose
                self.home_saved = True

        if self.received_msg is None:
            return Status.RUNNING

        try:
            data = json.loads(self.received_msg.data)
            bb['patient_id'] = data.get("patient_id", "Unknown")

            raw_depts = data.get("departments", DEFAULT_DEPARTMENTS)

            # 유효성 검사 + 안내데스크 제외
            depts = [
                d for d in raw_depts
                if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)
            ]

            bb['department_queue'] = list(depts)
            bb['remaining_depts'] = list(depts)
            bb['speak_text'] = "접수가 완료되었습니다. 이동을 시작할게요."

            print(f"[WaitForQR] 📋 환자: {bb['patient_id']}, 방문할 곳: {bb['remaining_depts']}")

            self.received_msg = None
            self.done = True
            return Status.SUCCESS

        except Exception as e:
            print(f"[WaitForQR] ⚠️ JSON 파싱 에러: {e}")
            self.received_msg = None
            return Status.RUNNING


class Think(SyncAction):
    """다음에 방문할 진료과를 결정 (대기인원 최소)"""
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.wait_min = 0
        self.wait_max = 20

    def _tick(self, agent, bb):
        remaining = bb.get('remaining_depts', []) or []

        # 안전장치: 안내데스크 제외
        if INFO_DESK_NAME in remaining:
            remaining = [d for d in remaining if d != INFO_DESK_NAME]
            bb['remaining_depts'] = remaining

        if len(remaining) == 0:
            print("[Think] 🎉 모든 진료과 방문 완료")
            return Status.FAILURE  # 루프 종료 트리거

        waiting_counts = {d: random.randint(self.wait_min, self.wait_max) for d in remaining}
        min_wait = min(waiting_counts.values())
        candidates = [d for d, w in waiting_counts.items() if w == min_wait]
        next_dept = random.choice(candidates)

        coords = DEPARTMENT_COORDINATES.get(next_dept)
        if not coords:
            print(f"[Think] ⚠️ 좌표 없음, 스킵: {next_dept}")
            remaining.remove(next_dept)
            bb['remaining_depts'] = remaining
            return Status.RUNNING

        bb['current_target_name'] = next_dept
        bb['current_target_coords'] = coords

        remaining.remove(next_dept)
        bb['remaining_depts'] = remaining

        bb['speak_text'] = f"{next_dept}로 이동할게요. 대기인원 {waiting_counts[next_dept]}명."
        print(f"[Think] 🧠 결정: {next_dept} (대기: {waiting_counts[next_dept]}명)")
        return Status.SUCCESS


class Move(ActionWithROSAction):
    """지정된 좌표로 이동하는 Nav2 액션 노드"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        target_name = bb.get('current_target_name', '알 수 없음')
        if not coords:
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))

        print(f"[Move] 🚀 {target_name}로 이동 시작 (x:{coords['x']}, y:{coords['y']})")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        target_name = bb.get('current_target_name', '목적지')

        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print(f"[Move] ✅ {target_name} 도착 확인 (SUCCEEDED)")
            bb['speak_text'] = f"{target_name}에 도착했습니다."
            return Status.SUCCESS

        print(f"[Move] ❌ {target_name} 이동 실패/취소 (Status Code: {status_code})")
        bb['speak_text'] = f"{target_name}로 이동하지 못했습니다."
        return Status.FAILURE


class WaitDoctorDone(SyncAction):
    """진료 완료 버튼(/hospital/doctor_input) 대기 노드"""
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self._done = False
        self.sub = agent.ros_bridge.node.create_subscription(
            Bool, "/hospital/doctor_input", self._cb, 10
        )

    def _cb(self, msg: Bool):
        if msg.data is True:
            print("[WaitDoctorDone] 👨‍⚕️ 의사 입력 수신됨!")
            self._done = True

    def _tick(self, agent, bb):
        if not self._done:
            return Status.RUNNING

        self._done = False
        bb['speak_text'] = "다음 진료과로 이동할게요."
        return Status.SUCCESS


class ReturnHome(ActionWithROSAction):
    """모든 일정이 끝나고 안내데스크로 복귀"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = DEPARTMENT_COORDINATES.get(INFO_DESK_NAME)
        if not coords:
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))

        print("[ReturnHome] 🏠 모든 업무 종료, 안내데스크로 복귀합니다.")
        return goal


class SpeakAction(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (speakActionMsg, 'speak_text'))

    def _build_goal(self, agent, bb):
        text_to_speak = bb.pop('speak_text', None)
        if not text_to_speak:
            return None

        goal = speakActionMsg.Goal()
        goal.text = text_to_speak
        print(f"[Speak] 🗣️ 말하기: '{text_to_speak}'")
        return goal


class WaitSpeedOK(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.limit = 0.8
        self._odom = None
        self._warned = False
        self.sub = agent.ros_bridge.node.create_subscription(Odometry, "/odom", self._cb, 10)

    def _cb(self, msg: Odometry):
        self._odom = msg

    def _tick(self, agent, bb):
        if self._odom is None:
            return Status.SUCCESS

        v = abs(self._odom.twist.twist.linear.x)
        if v > self.limit:
            if not self._warned:
                bb['speak_text'] = f"속도가 빨라요. {self.limit} 이하로 부탁해."
                self._warned = True
            return Status.SUCCESS

        self._warned = False
        return Status.SUCCESS


# ==========================================
# Condition Nodes
# ==========================================
class IsEmergencyPressed(ConditionWithROSTopics):
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, agent, [(Bool, "/emergency_trigger", "emergency_flag")], **kwargs)

    async def run(self, agent, bb):
        if "emergency_flag" not in self._cache:
            return Status.FAILURE
        return Status.SUCCESS if self._cache["emergency_flag"].data else Status.FAILURE


class IsBatteryLow(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(Bool, "/battery_low", "battery_flag")])

    def _predicate(self, agent, bb):
        if "battery_flag" in self._cache and self._cache["battery_flag"].data:
            print("[Battery] 🪫 배터리 부족!")
            return True
        return False


# ==========================================
# Abort / Siren / Email
# ==========================================
class SetAbort(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        bb['abort'] = True
        bb['speak_text'] = "비상 호출이 감지됐어. 지금 복귀할게."
        print("[Abort] 🚨 비상 플래그 설정")
        return Status.SUCCESS


class CheckAbort(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        return Status.FAILURE if bb.get('abort', False) else Status.SUCCESS


class SendDiagnosisEmail(SyncAction):
    def __init__(self, name, agent, topic="/hospital/send_diagnosis_email", **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(String, topic, 10)

    def _tick(self, agent, bb):
        payload = {
            "patient_id": bb.get("patient_id", "Unknown"),
            "email": bb.get("patient_email") or bb.get("email"),
            "request": "send_diagnosis_email"
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        print(f"[Email] 📧 진료 기록 이메일 전송 요청: {msg.data}")
        return Status.SUCCESS


class ControlSiren(SyncAction):
    def __init__(self, name, agent, enable=True, **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(Bool, "/cmd_siren", 10)
        self.enable_siren = bool(enable)

        # XML에서 enable="true/false" 로 들어오는 경우 대비
        if 'enable' in kwargs:
            val = str(kwargs['enable']).lower()
            self.enable_siren = (val == 'true')

    def _tick(self, agent, bb):
        msg = Bool()
        msg.data = self.enable_siren
        self.pub.publish(msg)
        print(f"[Siren] 📢 사이렌 제어: {self.enable_siren}")
        return Status.SUCCESS


# ==========================================
# Control Nodes
# ==========================================
class KeepRunningUntilFailure(Node):
    """자식이 Failure를 반환할 때까지 계속 실행 (Loop)"""
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []

    async def run(self, agent, bb):
        if not self.children:
            return Status.FAILURE
        status = await self.children[0].run(agent, bb)
        if status == Status.FAILURE:
            return Status.FAILURE
        return Status.RUNNING


class ForceSuccess(Node):
    """자식이 FAILURE여도 SUCCESS로 바꿔서 트리가 안 깨지게"""
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []

    async def run(self, agent, bb):
        if not self.children:
            return Status.SUCCESS
        status = await self.children[0].run(agent, bb)
        if status == Status.RUNNING:
            return Status.RUNNING
        return Status.SUCCESS


# ==========================================
# BT 노드 등록
# ==========================================
CUSTOM_ACTION_NODES = [
    'WaitForQR', 'SpeakAction', 'Think', 'WaitSpeedOK', 'Move',
    'WaitDoctorDone', 'ReturnHome', 'GoToInfoDesk', 'SendDiagnosisEmail',
    'SetAbort', 'CheckAbort', 'ControlSiren',
]
CUSTOM_CONDITION_NODES = ['IsEmergencyPressed', 'IsBatteryLow']

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')
BTNodeList.CONTROL_NODES.append('ForceSuccess')

print(f"✅ 커스텀 노드 등록 완료: action={len(CUSTOM_ACTION_NODES)}, condition={len(CUSTOM_CONDITION_NODES)}")
    