import math
import json
from modules.base_bt_nodes import (
    BTNodeList, Status, SyncAction, Node, 
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel,
)
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics
# ROS 2 Messages
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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
            "/hospital/patient_data", 
            self._callback, 
            10
        )
        self.home_saved = False

    def _callback(self, msg):
        self.received_msg = msg
        print("[WaitForQR] QR 데이터 수신됨!")

    def _tick(self, agent, blackboard):
        # 1. 초기 위치 저장
        if not self.home_saved:
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                blackboard['home_pose'] = agent.robot_pose
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
            blackboard['department_queue'] = dept_list
            blackboard['patient_id'] = data.get("patient_id", "Unknown")
            
            print(f"[WaitForQR] 데이터 확인됨. 환자: {blackboard['patient_id']}")
            
            self.received_msg = None 
            return Status.SUCCESS # 이제야 비로소 다음 단계로 이동 허가
            
        except json.JSONDecodeError:
            self.received_msg = None
            return Status.RUNNING
# ---------------------------------------------------------
# 2. Think: 다음 목적지 결정 (Iterator 역할)
# ---------------------------------------------------------
class Think(SyncAction):
    def __init__(self, name, agent):
        # [핵심 수정] 여기도 agent 대신 self._tick으로 변경 필수
        super().__init__(name, self._tick)

    def _tick(self, agent, blackboard):
        queue = blackboard.get('department_queue', [])
        
        if len(queue) > 0:
            next_dept = queue.pop(0)
            coords = DEPARTMENT_COORDINATES.get(next_dept)
            
            if coords:
                blackboard['current_target_name'] = next_dept
                blackboard['current_target_coords'] = coords
                blackboard['department_queue'] = queue
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

    def _build_goal(self, agent, blackboard):
        coords = blackboard.get('current_target_coords')
        if not coords:
            return None # 목표가 없으면 실행 안 함

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = float(coords['x'])
        goal.pose.pose.position.y = float(coords['y'])
        goal.pose.pose.orientation.w = 1.0 # 회전은 일단 정면 보기
        
        print(f"[Move] {blackboard.get('current_target_name')}로 이동 시작...")
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            print("[Move] 목적지 도착 완료.")
            return Status.SUCCESS
        else:
            print(f"[Move] 이동 실패 또는 취소됨 (Status: {status_code})")
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

    def _predicate(self, agent, blackboard):
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
    blackboard['home_pose']로 이동
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, 'navigate_to_pose'))

    def _build_goal(self, agent, blackboard):
        home_pose = blackboard.get('home_pose')
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
    async def run(self, agent, blackboard):
        if not self.children:
            return Status.FAILURE
            
        # 자식 노드의 run 함수를 비동기로 기다림 (await)
        status = await self.children[0].run(agent, blackboard)
        
        # 자식이 실패하면 -> 루프 종료 (나도 실패 반환)
        if status == Status.FAILURE:
            return Status.FAILURE
            
        # 자식이 성공했거나 실행 중이면 -> 나는 계속 실행 중(RUNNING) -> 다시 실행됨
        return Status.RUNNING
# ---------------------------------------------------------
# 노드 등록
# ---------------------------------------------------------
CUSTOM_ACTION_NODES = [
    'WaitForQR',
    'Think',
    'Move',
    'Doctor',
    'ReturnHome' # XML에서는 <return> 태그를 쓸 것이므로 매핑 주의
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONTROL_NODES.append('KeepRunningUntilFailure')