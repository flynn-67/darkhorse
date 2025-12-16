import math
import json
import random
import rclpy
from rclpy.node import Node
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics

# ROS 2 Messages
from limo_interfaces.action import Speak as speakActionMsg
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry


# ==========================================
# 상수 및 좌표 정의
# ==========================================
INFO_DESK_NAME = "안내데스크"

# 좌표 설정 (환경에 맞게 수정 필요)
DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": -2.0478696823120117, "y": 1.3148077726364136, "w": 1.0},
    "정형외과":      {"x": 4.325248718261719, "y": -1.067739486694336, "w": 1.0},
    "안내데스크":    {"x": 0.08828259259462357, "y": 0.08828259259462357, "w": 1.0},
}
DEFAULT_DEPARTMENTS = ["진단검사의학과", "정형외과"]

def publish_ui_status(ros_node, text):
    pub = ros_node.create_publisher(String, '/hospital/nav_status', 10)
    msg = String()
    msg.data = text
    pub.publish(msg)


# ==========================================
# Action Nodes
# ==========================================
class GoToInfoDesk(ActionWithROSAction):
    """안내데스크로 이동 (타임아웃 60초 + 비상 시 강제 성공)"""
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
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

        publish_ui_status(self.ros.node, "안내데스크 복귀 중 🏠")
        print("[GoToInfoDesk] 🏠 안내데스크로 복귀 시작")
        
        self.start_time = self.ros.node.get_clock().now()
        self.nav_goal_sent = True
        return goal

    async def run(self, agent, bb):
        status = await super().run(agent, bb)
        
        # 타임아웃 체크 (60초)
        if status == Status.RUNNING and self.nav_goal_sent:
            now = self.ros.node.get_clock().now()
            elapsed_time = (now - self.start_time).nanoseconds / 1e9
            
            if elapsed_time > self.timeout_sec:
                print(f"[GoToInfoDesk] ⚠️ 60초 타임아웃! 강제 종료.")
                if self._action_client and self._goal_handle:
                    self._action_client.cancel_goal_async(self._goal_handle)
                self.nav_goal_sent = False
                return Status.SUCCESS # 강제 성공 반환
            
        return status

    def _interpret_result(self, result, agent, bb, status_code=None):
        self.nav_goal_sent = False
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[GoToInfoDesk] ✅ 도착 완료")
            return Status.SUCCESS
        
        if bb.get('abort', False):
            print(f"[GoToInfoDesk] ⚠️ 비상 상황: 이동 실패했으나 성공 처리")
            publish_ui_status(self.ros.node, "복귀 완료 (강제)")
            return Status.SUCCESS
            
        print(f"[GoToInfoDesk] ❌ 이동 실패 (Code: {status_code})")
        return Status.FAILURE


class WaitForQR(SyncAction):
    """QR 대기 및 상태 초기화"""
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        self.done = False
        self.sub = agent.ros_bridge.node.create_subscription(String, "/hospital/qr_login", self._callback, 10)
        self.home_saved = False
        self.first_run = True

    def _callback(self, msg): self.received_msg = msg

    def _tick(self, agent, bb):
        if self.first_run:
            publish_ui_status(agent.ros_bridge.node, "환자 접수 대기 중... 📋")
            bb['abort'] = False  # ✅ 비상 상태 리셋
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
            bb['speak_text'] = "접수가 완료되었습니다."

            self.received_msg = None
            self.done = True
            publish_ui_status(agent.ros_bridge.node, f"환자 {bb['patient_id']} 접수 완료 ✅")
            return Status.SUCCESS
        except Exception as e:
            self.received_msg = None
            return Status.RUNNING


WAITING_TOPIC = "/hospital/waiting_board"

class Think(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        remaining = bb.get('remaining_depts', []) or []
        if INFO_DESK_NAME in remaining:
            remaining = [d for d in remaining if d != INFO_DESK_NAME]

        # ✅ 갈 곳이 없으면 FAILURE 반환 → 루프 종료 신호
        if len(remaining) == 0:
            return Status.FAILURE

        dept_wait = bb.get("dept_wait", {}) or {}

        # ✅ remaining 중에서 "UI에서 받은 대기정보"가 있는 과만 후보로
        candidates = []
        for d in remaining:
            if d in dept_wait:
                try:
                    w = int(dept_wait[d])
                    candidates.append((d, w))
                except:
                    pass

        # ✅ 토픽이 아직 안 들어왔거나 후보가 비면 fallback
        if not candidates:
            next_dept = random.choice(remaining)
        else:
            min_wait = min(w for _, w in candidates)
            tied = [d for d, w in candidates if w == min_wait]
            next_dept = random.choice(tied)

        coords = DEPARTMENT_COORDINATES.get(next_dept)
        if not coords:
            # 좌표 없으면 남은 목록에서 제외하고 다음 tick에서 재선정
            if next_dept in remaining:
                remaining.remove(next_dept)
            bb['remaining_depts'] = remaining
            return Status.RUNNING

        bb['current_target_name'] = next_dept
        bb['current_target_coords'] = coords
        if next_dept in remaining:
            remaining.remove(next_dept)
        bb['remaining_depts'] = remaining

        bb['speak_text'] = f"{next_dept}로 이동할게요."
        return Status.SUCCESS

class WaitingBoardSub(Node):
    """
    UI(전광판)에서 발행하는 /hospital/waiting_board 를 구독해서
    블랙보드(bb)에 dept_wait / dept_queue를 반영해주는 노드
    """
    def __init__(self, bb):
        super().__init__("waiting_board_sub")
        self.bb = bb
        self.create_subscription(String, WAITING_TOPIC, self.cb, 10)

    def cb(self, msg):
        try:
            data = json.loads(msg.data)

            dept_wait = data.get("dept_wait", {}) or {}
            dept_queue = data.get("dept_queue", {}) or {}

            # ✅ 타입 정리(혹시 값이 문자열로 오더라도)
            cleaned_wait = {}
            for k, v in dept_wait.items():
                try:
                    cleaned_wait[str(k)] = int(v)
                except:
                    continue

            cleaned_queue = {}
            for k, v in dept_queue.items():
                if isinstance(v, list):
                    cleaned_queue[str(k)] = [str(x) for x in v]
                else:
                    cleaned_queue[str(k)] = []

            self.bb["dept_wait"] = cleaned_wait
            self.bb["dept_queue"] = cleaned_queue
            self.bb["waiting_ts"] = int(data.get("ts", 0))

        except Exception as e:
            self.get_logger().warn(f"bad waiting_board payload: {e}")
class Move(ActionWithROSAction):
    def __init__(self, name, agent): super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        target_name = bb.get('current_target_name', '목적지')
        if not coords: return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = float(coords.get('w', 1.0))
        publish_ui_status(self.ros.node, f"{target_name} 이동 중 🚑")
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
        if msg.data is True: self._done = True
    def _tick(self, agent, bb):
        if not self.status_sent:
            target_name = bb.get('current_target_name', '진료과')
            publish_ui_status(agent.ros_bridge.node, f"{target_name} 진료 중... 👨‍⚕️")
            self.status_sent = True
        if not self._done: return Status.RUNNING
        self._done = False; self.status_sent = False
        bb['speak_text'] = "진료 종료. 다음으로 이동."
        return Status.SUCCESS


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


class IsEmergencyPressed(ConditionWithROSTopics):
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, agent, [(Bool, "/emergency_trigger", "emergency_flag")], **kwargs)
    async def run(self, agent, bb):
        if bb.get('abort', False): return Status.SUCCESS # Latch
        if "emergency_flag" not in self._cache: return Status.FAILURE
        return Status.SUCCESS if self._cache["emergency_flag"].data else Status.FAILURE


class IsBatteryLow(ConditionWithROSTopics):
    def __init__(self, name, agent): super().__init__(name, agent, [(Bool, "/battery_low", "battery_flag")])
    def _predicate(self, agent, bb):
        return "battery_flag" in self._cache and self._cache["battery_flag"].data


class SetAbort(SyncAction):
    def __init__(self, name, agent): super().__init__(name, self._tick)
    def _tick(self, agent, bb):
        bb['abort'] = True
        bb['speak_text'] = "비상 상황 발생! 복귀합니다."
        print("[Abort] 🚨 비상 플래그 설정")
        return Status.SUCCESS


# ✅ [NotAbort] 이름 유지 (XML과 통일)
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
        payload = {"patient_id": bb.get("patient_id", "Unknown"), "email": bb.get("patient_email"), "request": "send_diagnosis_email"}
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
        return Status.SUCCESS


class ReturnHome(ActionWithROSAction): # Placeholder for class compatibility
    def __init__(self, name, agent): super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))
    def _build_goal(self, agent, bb): return None


# ==========================================
# Control Nodes (핵심 수정!)
# ==========================================
class KeepRunningUntilFailure(Node):
    """자식이 Failure(루프 종료)를 반환하면 SUCCESS로 변환하여 다음 단계로 넘김"""
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []

    async def run(self, agent, bb):
        if not self.children: return Status.FAILURE
        status = await self.children[0].run(agent, bb)
        
        # ✅ 자식이 FAILURE(갈 곳 없음 or NotAbort) -> 루프 정상 종료(SUCCESS)
        if status == Status.FAILURE:
            return Status.SUCCESS
            
        # 자식이 SUCCESS -> 계속 반복(RUNNING)
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

print(f"✅ 커스텀 노드 등록 완료")
