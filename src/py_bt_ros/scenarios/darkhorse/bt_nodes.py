import math
import json
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node, 
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics
# ROS 2 Messages
from limo_interfaces.action import Speak as speakActionMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

# bb = blackboard 
# ---------------------------------------------------------
# 병원 진료과 좌표 매핑 (예시 - 실제 맵 좌표에 맞게 수정 필요)
# ---------------------------------------------------------
DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48, "y": 0.27, "w": 1.0},
    "영상의학과":    {"x": 6.57, "y": 2.62, "w": 1.0},
    "내과":          {"x": 7.44, "y": 0.51, "w": 1.0},
    "정형외과":      {"x": 0.75, "y": -2.64, "w": 1.0},
    "신경과":        {"x": 2.83, "y": 1.17, "w": 1.0}
}

# ---------------------------------------------------------
# 1. WaitForQR: QR(JotForm) 데이터 수신 및 경로 계획
# ---------------------------------------------------------
# [중요] 상속 변경: ConditionWithROSTopics -> Node
# 우리는 단순히 True/False 체크가 아니라, 데이터가 올 때까지 '대기(Running)' 상태를 유지해야 합니다.
# [수정된 WaitForQR 전체 코드]
# [수정 1] Node -> SyncAction 으로 변경 (run 함수 자동 지원)
class WaitForQR(SyncAction):
    def __init__(self, name, agent):
        # [핵심 수정 1] agent 대신 self._tick을 넘겨야 에러가 안 납니다!
        super().__init__(name, self._tick)
        self.agent = agent
        self.received_msg = None
        
        # [핵심 수정 2] agent.ros_bridge.node 경로 사용
        self.sub = agent.ros_bridge.node.create_subscription(
            String, 
            "/hospital/system_start", 
            self._callback, 
            10
        )
        self.home_saved = False

    def _callback(self, msg):
        self.received_msg = msg
        print("[WaitForQR] QR 데이터 수신됨!")

    def _tick(self, agent, bb):
        # 1. 초기 위치 저장
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                bb['home_pose'] = agent.robot_pose
                bb['speak_text'] = "초기 위치가 저장되었습니다. 진료과로 이동을 시작합니다."
                self.home_saved = True
                print(f"[WaitForQR] 초기 위치 저장 완료")

        # 2. 메시지가 없으면 -> 절대 움직이지 마! (RUNNING 반환)
        if self.received_msg is None:
            # 여기서 RUNNING을 반환해야 트리가 다음 단계(Think/Move)로 안 넘어갑니다.
            return Status.RUNNING

        # 3. 메시지 처리
        try:
            data = json.loads(self.received_msg.data)
            dept_list = data.get("departments", [])
            bb['department_queue'] = dept_list
            bb['patient_id'] = data.get("patient_id", "Unknown")
            
            print(f"[WaitForQR] 데이터 확인됨. 환자: {bb['patient_id']}")
            
            self.received_msg = None 
            return Status.SUCCESS # 이제야 비로소 다음 단계로 이동 허가
            
        except json.JSONDecodeError:
            self.received_msg = None
            return Status.RUNNING
        
# ---------------------------------------------------------
# [추가] Condition Nodes: 상태 체크용 노드
# ---------------------------------------------------------
class IsEmergencyPressed(ConditionWithROSTopics):
    """
    비상 정지 버튼이 눌렸는지 확인하는 노드
    토픽: /emergency_stop (Bool) -> True이면 비상 상황
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, "/emergency_stop", "emergency_flag")
        ])

    def _predicate(self, agent, bb):
        # 메시지가 캐시에 있으면 확인
        if "emergency_flag" in self._cache:
            is_pressed = self._cache["emergency_flag"].data
            if is_pressed:
                print("[Emergency] 비상 정지 버튼 감지됨!")
                return True # 비상 상황임 (Condition True)
        
        return False # 비상 상황 아님

class IsBatteryLow(ConditionWithROSTopics):
    """
    배터리가 부족한지 확인하는 노드
    토픽: /battery_status (가정, 로봇에 맞게 수정 필요)
    여기서는 테스트를 위해 /battery_low (Bool) 토픽을 구독한다고 가정
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, "/battery_low", "battery_flag")
        ])

    def _predicate(self, agent, bb):
        if "battery_flag" in self._cache:
            is_low = self._cache["battery_flag"].data
            if is_low:
                print("[Battery] 배터리 부족 감지!")
                return True
        return False
# ---------------------------------------------------------
# 2. Think: 다음 목적지 결정 (Iterator 역할)
# ---------------------------------------------------------
class Think(SyncAction):
    def __init__(self, name, agent):
        # [핵심 수정] 여기도 agent 대신 self._tick으로 변경 필수
        super().__init__(name, self._tick)

    def _tick(self, agent, bb):
        queue = bb.get('department_queue', [])
        
        if len(queue) > 0:
            next_dept = queue.pop(0)
            coords = DEPARTMENT_COORDINATES.get(next_dept)
            
            if coords:
                bb['current_target_name'] = next_dept
                bb['current_target_coords'] = coords
                bb['department_queue'] = queue
                print(f"[Think] 다음 목적지: {next_dept}")
                return Status.SUCCESS
            else:
                print(f"[Think] 좌표 없음: {next_dept}")
                return Status.FAILURE
        else:
            print("[Think] 모든 진료과 방문 완료.")
            return Status.FAILURE
# ---------------------------------------------------------
# 3. Move: Nav2 Action을 이용한 이동
# ---------------------------------------------------------
class Move(ActionWithROSAction):
    """
    blackboard['current_target_coords']로 이동 (Nav2 NavigateToPose)
    """
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        # Nav2의 기본 액션 토픽: /navigate_to_pose
        super().__init__(name, agent, (NavigateToPose, 'navigate_to_pose'))

    def _build_goal(self, agent, bb):
        coords = bb.get('current_target_coords')
        if not coords:
            return None # 목표가 없으면 실행 안 함

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = 1.0 # 회전은 일단 정면 보기
        
        print(f"[Move] {bb.get('current_target_name')}로 이동 시작...")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[Move] 목적지 도착 완료.")
            bb['speak_text'] = f"{bb.get('current_target_name', '목적지')}에 도착했습니다."
            return Status.SUCCESS
        else:
            print(f"[Move] 이동 실패 또는 취소됨 (Status: {status_code})")
            bb['speak_text'] = f"{bb.get('current_target_name', '목적지')}로 이동에 실패 또는 취소됬습니다."
            return Status.FAILURE

# ---------------------------------------------------------
# 4. Doctor: 의료진 대시보드 입력 대기
# ---------------------------------------------------------
class Doctor(ConditionWithROSTopics):
    """
    의료진이 진단을 완료하고 '다음' 버튼을 누르면 메시지를 보낸다고 가정.
    토픽: /hospital/doctor_input (Bool)
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, "/hospital/doctor_input", "doctor_signal")
        ])

    def _predicate(self, agent, bb):
        # 메시지가 들어왔는지 확인
        if "doctor_signal" in self._cache:
            msg = self._cache["doctor_signal"]
            if msg.data is True:
                print("[Doctor] 진료 완료 확인. 다음 단계로.")
                del self._cache["doctor_signal"] # 사용한 신호 삭제
                return True
        
        # 메시지 올 때까지 대기 (RUNNING)
        return False

# ---------------------------------------------------------
# 5. Return: 초기 위치로 복귀
# ---------------------------------------------------------
class ReturnHome(ActionWithROSAction):
    """
    블랙보드['home_pose']로 이동
    bb['home_pose']로 이동
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, 'navigate_to_pose'))

    def _build_goal(self, agent, bb):
        home_pose = bb.get('home_pose')
        if not home_pose:
            # 홈 위치가 없으면 (0,0)으로
            home_pose = PoseStamped()
            home_pose.pose.position.x = 0.0
            home_pose.pose.position.y = 0.0
            home_pose.pose.orientation.w = 1.0
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        # 저장된 home_pose가 Pose 객체일 수 있으므로 상황에 맞게 매핑
        goal.pose.pose = home_pose if hasattr(home_pose, 'position') else home_pose.pose

        print("[Return] 모든 일정을 마치고 초기 위치로 복귀합니다.")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


class KeepRunningUntilFailure(Node):
    def __init__(self, name, children=None):
        super().__init__(name)
        self.children = children if children is not None else []

    # 중요: 비동기(async) 실행 함수로 만들어야 함
    async def run(self, agent, bb):
        if not self.children:
            return Status.FAILURE
            
        # 자식 노드의 run 함수를 비동기로 기다림 (await)
        status = await self.children[0].run(agent, bb)
        
        # 자식이 실패하면 -> 루프 종료 (나도 실패 반환)
        if status == Status.FAILURE:
            return Status.FAILURE
            
        # 자식이 성공했거나 실행 중이면 -> 나는 계속 실행 중(RUNNING) -> 다시 실행됨
        return Status.RUNNING

# ---------------------------------------------------------
# 6. SpeakAction: TTS 액션 노드
# ---------------------------------------------------------
class SpeakAction(ActionWithROSAction):
    """
    Docstring for SpeakActon
    TTS 액션 서버 노드: limo_tts/limo_tts/speak_text 사용
    added date 2025/dec/14 by Raybeak    
    """
    def __init__ (self, name, agent):
        super().__init__(name, agent, (speakActionMsg, 'speak_text'))

    def _build_goal(self, agent, bb):
        #text_to_speak = f"{bb.get('current_target_name', '알 수 없음')} 에 도착 했습니다."
       
        text_to_speak = bb.pop('speak_text', None)
        if not text_to_speak:
            return Status.SUCCESS  # 말할 내용이 없으면 실행 안 함
        
        goal = speakActionMsg.Goal()
        goal.text = text_to_speak
        print(f"[Speak] TTS 요청: {text_to_speak}")
        return goal   
    def _tick(self, agent, bb):
        # 1. Goal 객체 생성
        goal_msg = Speak.Goal()
        
        # 2. 텍스트 할당 (blackboard에서 가져오거나 직접 입력)
        goal_msg.text = "환자를 확인했습니다"  # 혹은 bb['tts_text']
        
        # 3. Goal 객체를 전송 (문자열이 아닌 goal_msg를 보내야 함)
        # base_bt_nodes_ros.py가 이 goal_msg를 받도록 수정되어야 합니다.
        self.client.send_goal_async(goal_msg).add_done_callback(self._on_goal_response)
        
        return Status.RUNNING

class MonitorSpeed(ConditionWithROSTopics):
    """
    로봇의 속도를 모니터링하는 노드
    토픽: /odom (nav_msgs/Odometry)
    기능: 속도가 제한 속도(예: 0.8 m/s) 이하이면 SUCCESS(True), 초과하면 FAILURE(False)
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Odometry, "/odom", "odom_msg")
        ])
        self.limit = 0.8  # 제한 속도 설정 (m/s)

    def _predicate(self, agent, bb):
        if "odom_msg" in self._cache:
            odom = self._cache["odom_msg"]
            # 선속도(linear.x) 확인
            current_speed = abs(odom.twist.twist.linear.x)
            
            if current_speed > self.limit:
                print(f"[MonitorSpeed] 과속 감지! 현재 속도: {current_speed:.2f} m/s")
                return False  # 과속이면 실패 처리 (-> Fallback에서 Stop 동작 유도 등)
            
        # 데이터가 없거나 속도가 정상이면 통과
        return True 

# ---------------------------------------------------------
# 노드 등록 (수정본)
# ---------------------------------------------------------

# 1. 액션 노드 이름 등록
CUSTOM_ACTION_NODES = [
    'WaitForQR',
    'SpeakAction',
    'Think',
    'Move',
    'ReturnHome'
]

# 2. 조건 노드 이름 등록 (IsEmergencyPressed 추가!)
CUSTOM_CONDITION_NODES = [
    'IsEmergencyPressed',
    'IsBatteryLow',
    'Doctor',
        # Doctor는 입력을 기다리는 조건(Condition) 성격이 강하므로 여기 넣는 게 맞을 수도 있습니다.
              # (위에서 ConditionWithROSTopics를 상속받았다면 여기에 넣으세요.)
    'MonitorSpeed',
]

# 3. BTNodeList에 확장 (Extend)
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES) # 이 줄이 꼭 있어야 합니다!
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')

print(f"Registered Actions: {BTNodeList.ACTION_NODES}")
print(f"Registered Conditions: {BTNodeList.CONDITION_NODES}")