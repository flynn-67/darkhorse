import math
import json
import random
import os
from pathlib import Path
import yaml

from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics

from limo_interfaces.action import Speak as speakActionMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

INFO_DESK_NAME = "안내데스크"

# =========================
# YAML waypoint loader (bt_nodes)
# =========================
_WP_CACHE = {"path": None, "mtime": None, "depts": {}}

def _find_waypoint_yaml(default_name="hospital_waypoints.yaml") -> str:
    # 1) ENV 우선
    env = os.environ.get("HOSPITAL_WAYPOINTS_FILE")
    if env:
        return env

    # 2) ~/.ros fallback
    cand = os.path.expanduser("~/.ros/hospital_waypoints.yaml")
    if os.path.exists(cand):
        return cand

    # 3) 현재 파일 기준으로 위로 올라가며 config/ 찾아보기
    here = Path(__file__).resolve()
    for p in [here.parent] + list(here.parents):
        c = p / "config" / default_name
        if c.exists():
            return str(c)

    # 마지막 fallback
    return cand

WAYPOINT_FILE = _find_waypoint_yaml()

def _reload_waypoints(force: bool = False):
    global _WP_CACHE
    path = WAYPOINT_FILE

    try:
        mtime = os.path.getmtime(path) if os.path.exists(path) else None
    except Exception:
        mtime = None

    if (not force) and (_WP_CACHE["path"] == path) and (_WP_CACHE["mtime"] == mtime) and (_WP_CACHE["depts"] is not None):
        return

    if not os.path.exists(path):
        _WP_CACHE = {"path": path, "mtime": mtime, "depts": {}}
        return

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        depts = data.get("departments", {}) or {}

        cleaned = {}
        for name, info in depts.items():
            if isinstance(info, dict) and ("x" in info) and ("y" in info):
                cleaned[str(name)] = dict(info)

        _WP_CACHE = {"path": path, "mtime": mtime, "depts": cleaned}
    except Exception:
        # 파싱 실패 시 기존 캐시 유지 (갑자기 비워지면 위험해서)
        return

def get_coords(name: str):
    _reload_waypoints(force=False)
    return _WP_CACHE["depts"].get(name)

def list_departments(exclude_info_desk: bool = True):
    _reload_waypoints(force=False)
    names = list(_WP_CACHE["depts"].keys())
    if exclude_info_desk:
        names = [n for n in names if n != INFO_DESK_NAME]
    return names

def _yaw_to_quat(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

def _apply_orientation_from_coords(pose, coords: dict):
    """
    coords에 yaw가 있으면 yaw -> quaternion 적용
    없으면 기존 호환: z,w 또는 w만 사용
    """
    if coords is None:
        return
    if "yaw" in coords:
        _, _, qz, qw = _yaw_to_quat(float(coords["yaw"]))
        pose.orientation.z = qz
        pose.orientation.w = qw
    else:
        pose.orientation.z = float(coords.get("z", 0.0))
        pose.orientation.w = float(coords.get("w", 1.0))


# =========================
# BT Nodes
# =========================
class GoToInfoDesk(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = get_coords(INFO_DESK_NAME)
        if not coords:
            print("[GoToInfoDesk] 안내데스크 좌표가 YAML에 없음!")
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        _apply_orientation_from_coords(goal.pose.pose, coords)

        print(f"[GoToInfoDesk] 🚨 비상 상황! 안내데스크({coords})로 이동합니다.")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[GoToInfoDesk] 안내데스크 도착 완료.")
            return Status.SUCCESS
        return Status.FAILURE


class WaitForQR(SyncAction):
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
        print("[WaitForQR] QR 데이터 수신됨!")

    def _tick(self, agent, bb):
        if self.done:
            return Status.SUCCESS

        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                bb['home_pose'] = agent.robot_pose
                self.home_saved = True

        if self.received_msg is None:
            return Status.RUNNING

        try:
            data = json.loads(self.received_msg.data)

            bb['patient_id'] = data.get("patient_id", "Unknown")

            raw_depts = data.get("departments", None)

            # ✅ QR에 과 목록이 없으면: YAML에 있는 과 목록(안내데스크 제외)을 기본값으로 사용
            if not raw_depts:
                raw_depts = list_departments(exclude_info_desk=True) or ["진단검사의학과", "영상의학과"]

            # ✅ 유효 과만 남기되, 안내데스크는 후보에서 제외
            available = set(list_departments(exclude_info_desk=False))
            depts = [
                d for d in raw_depts
                if (d in available) and (d != INFO_DESK_NAME)
            ]

            bb['department_queue'] = list(depts)
            bb['remaining_depts']  = list(depts)

            bb['speak_text'] = "접수가 완료되었습니다. 이동을 시작할게요."

            print(f"[WaitForQR] 환자: {bb['patient_id']}")
            print(f"[WaitForQR] 기본/QR 과 목록: {raw_depts}")
            print(f"[WaitForQR] 유효 과 목록(안내데스크 제외): {bb['remaining_depts']}")

            self.received_msg = None
            self.done = True
            return Status.SUCCESS

        except Exception as e:
            print("[WaitForQR] parse fail:", e)
            self.received_msg = None
            return Status.RUNNING


class IsEmergencyPressed(ConditionWithROSTopics):
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, agent, [(Bool, "/emergency_trigger", "emergency_flag")], **kwargs)

    async def run(self, agent, bb):
        if "emergency_flag" not in self._cache:
            self.status = Status.FAILURE
            return self.status

        is_pressed = self._cache["emergency_flag"].data
        self.status = Status.SUCCESS if is_pressed else Status.FAILURE
        return self.status


class IsBatteryLow(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [(Bool, "/battery_low", "battery_flag")])

    def _predicate(self, agent, bb):
        if "battery_flag" in self._cache and self._cache["battery_flag"].data:
            print("[Battery] 배터리 부족 감지!")
            return True
        return False


class Think(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.wait_min = 0
        self.wait_max = 20

    def _tick(self, agent, bb):
        remaining = bb.get('remaining_depts', []) or []

        # ✅ 안전장치: 혹시 남아있으면 안내데스크 제거
        if INFO_DESK_NAME in remaining:
            remaining = [d for d in remaining if d != INFO_DESK_NAME]
            bb['remaining_depts'] = remaining

        print("[Think DEBUG] remaining_depts =", remaining)

        if len(remaining) == 0:
            print("[Think] 모든 진료과 방문 완료.")
            return Status.FAILURE

        waiting_counts = {d: random.randint(self.wait_min, self.wait_max) for d in remaining}

        min_wait = min(waiting_counts.values())
        candidates = [d for d, w in waiting_counts.items() if w == min_wait]
        next_dept = random.choice(candidates)

        coords = get_coords(next_dept)
        if not coords:
            print(f"[Think] YAML에 좌표 없음: {next_dept} -> 후보에서 제거")
            remaining.remove(next_dept)
            bb['remaining_depts'] = remaining
            return Status.RUNNING

        bb['current_target_name'] = next_dept
        bb['current_target_coords'] = coords

        remaining.remove(next_dept)
        bb['remaining_depts'] = remaining

        bb['speak_text'] = f"{next_dept}로 이동할게요. 대기인원 {waiting_counts[next_dept]}명."

        print(f"[Think] 후보 대기: {waiting_counts}")
        print(f"[Think] 선택: {next_dept} (wait={waiting_counts[next_dept]})")
        return Status.SUCCESS


class Move(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        if not coords:
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        _apply_orientation_from_coords(goal.pose.pose, coords)

        print(f"[Move] {bb.get('current_target_name')}로 이동 시작...")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[Move] 목적지 도착 완료.")
            bb['speak_text'] = f"{bb.get('current_target_name', '목적지')}에 도착했습니다."
            return Status.SUCCESS

        print(f"[Move] 이동 실패 또는 취소됨 (Status: {status_code})")
        bb['speak_text'] = f"{bb.get('current_target_name', '목적지')}로 이동에 실패 또는 취소됬습니다."
        return Status.FAILURE


class WaitDoctorDone(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self._done = False
        self.sub = agent.ros_bridge.node.create_subscription(
            Bool, "/hospital/doctor_input", self._cb, 10
        )

    def _cb(self, msg: Bool):
        if msg.data is True:
            self._done = True

    def _tick(self, agent, bb):
        if not self._done:
            return Status.RUNNING

        self._done = False
        bb['speak_text'] = "다음 진료과로 이동할게요."
        return Status.SUCCESS


class ReturnHome(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, '/navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = get_coords(INFO_DESK_NAME)
        if not coords:
            print("[ReturnHome] 안내데스크 좌표가 YAML에 없음!")
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        _apply_orientation_from_coords(goal.pose.pose, coords)

        print("[Return] 안내데스크로 복귀합니다.")
        return goal


class KeepRunningUntilFailure(Node):
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


class SpeakAction(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, (speakActionMsg, 'speak_text'))

    def _build_goal(self, agent, bb):
        text_to_speak = bb.pop('speak_text', None)
        if not text_to_speak:
            return None

        goal = speakActionMsg.Goal()
        goal.text = text_to_speak
        print(f"[Speak] TTS 요청: {text_to_speak}")
        return goal


class WaitSpeedOK(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)
        self.limit = 0.8
        self._odom = None
        self._warned = False
        self.sub = agent.ros_bridge.node.create_subscription(
            Odometry, "/odom", self._cb, 10
        )

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


class SetAbort(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        bb['abort'] = True
        bb['speak_text'] = "비상 호출이 감지됐어. 지금 복귀할게."
        return Status.SUCCESS


class CheckAbort(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        if bb.get('abort', False):
            return Status.FAILURE
        return Status.SUCCESS


class SendDiagnosisEmail(SyncAction):
    def __init__(self, name, agent, topic="/hospital/send_diagnosis_email", **kwargs):
        super().__init__(name, self._tick, **kwargs)
        self.ros = agent.ros_bridge
        self.pub = self.ros.node.create_publisher(String, topic, 10)
        self.topic = topic

    def _tick(self, agent, bb):
        payload = {
            "patient_id": bb.get("patient_id", "Unknown"),
            "email": bb.get("patient_email") or bb.get("email"),
            "request": "send_diagnosis_email"
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        print(f"[SendDiagnosisEmail] published -> {self.topic}: {msg.data}")
        return Status.SUCCESS


CUSTOM_ACTION_NODES = [
    'WaitForQR',
    'SpeakAction',
    'Think',
    'WaitSpeedOK',
    'Move',
    'WaitDoctorDone',
    'ReturnHome',
    'GoToInfoDesk',
    'SendDiagnosisEmail',
    'SetAbort',
    'CheckAbort',
]

CUSTOM_CONDITION_NODES = [
    'IsEmergencyPressed',
    'IsBatteryLow',
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')

print(f"Registered Actions: {BTNodeList.ACTION_NODES}")
print(f"Registered Conditions: {BTNodeList.CONDITION_NODES}")
