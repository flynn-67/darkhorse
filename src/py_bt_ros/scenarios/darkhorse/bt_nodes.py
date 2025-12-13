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
    "내과": {"x": 1.0, "y": 0.5},
    "정형외과": {"x": 2.5, "y": 1.0},
    "영상의학과": {"x": 0.0, "y": 2.0},
    "신경과": {"x": -1.5, "y": 1.0},
    "진단검사의학과": {"x": -0.5, "y": -1.0}
}

# ---------------------------------------------------------
# 1. WaitForQR: QR(JotForm) 데이터 수신 및 경로 계획
# ---------------------------------------------------------
class WaitForQR(ConditionWithROSTopics):
    """
    JotForm 데이터(/hospital/patient_data)를 기다림.
    데이터 포맷 예시(JSON string): {"patient_id": "123", "departments": ["내과", "영상의학과"]}
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (String, "/hospital/patient_data", "patient_msg")
        ])
        # 시작 위치 저장을 위한 플래그
        self.home_saved = False

    def _predicate(self, agent, blackboard):
        # 1. 현재 로봇 위치를 Home으로 저장 (한 번만 실행)
        if not self.home_saved:
            # agent.robot_pose는 로봇의 현재 위치(Pose)를 가지고 있다고 가정 (ros_bridge에서 갱신됨)
            # 만약 없다면 별도 tf 리스너나 odom 구독이 필요하지만, 여기서는 agent가 pose를 안다고 가정
            if hasattr(agent, 'robot_pose') and agent.robot_pose is not None:
                blackboard['home_pose'] = agent.robot_pose
                self.home_saved = True
            else:
                # 로봇 위치를 아직 모르면 일단 (0,0)으로 가정하거나 대기
                pass

        # 2. 메시지 수신 확인
        if "patient_msg" not in self._cache:
            return False # 아직 메시지 안 옴 (RUNNING)

        msg = self._cache["patient_msg"] # std_msgs/String
        try:
            data = json.loads(msg.data)
            dept_list = data.get("departments", [])
            
            # 방문해야 할 진료과 리스트를 큐(Queue) 형태로 블랙보드에 저장
            blackboard['department_queue'] = dept_list
            blackboard['patient_id'] = data.get("patient_id", "Unknown")
            
            print(f"[WaitForQR] 환자({blackboard['patient_id']}) 데이터 수신. 방문 예정: {dept_list}")
            
            # 처리가 끝났으므로 캐시 비우고 SUCCESS 반환
            del self._cache["patient_msg"]
            return True 
            
        except json.JSONDecodeError:
            print("[WaitForQR] JSON 파싱 에러")
            return False

# ---------------------------------------------------------
# 2. Think: 다음 목적지 결정 (Iterator 역할)
# ---------------------------------------------------------
class Think(SyncAction):
    """
    department_queue에서 하나를 꺼내 현재 목표(target_pose)로 설정.
    큐가 비어있으면 FAILURE를 반환하여 루프 종료를 알림.
    """
    def __init__(self, name, agent):
        super().__init__(name, agent)

    def _tick(self, agent, blackboard):
        queue = blackboard.get('department_queue', [])
        
        if len(queue) > 0:
            # 다음 진료과 꺼내기
            next_dept = queue.pop(0)
            coords = DEPARTMENT_COORDINATES.get(next_dept)
            
            if coords:
                blackboard['current_target_name'] = next_dept
                blackboard['current_target_coords'] = coords
                blackboard['department_queue'] = queue # 업데이트된 큐 저장
                print(f"[Think] 다음 목적지 설정: {next_dept} {coords}")
                return Status.SUCCESS
            else:
                print(f"[Think] 경고: {next_dept}의 좌표 정보가 없습니다. 스킵합니다.")
                # 좌표가 없으면 재귀적으로 다음 것 찾거나 실패 처리 (여기서는 실패 처리)
                return Status.FAILURE
        else:
            print("[Think] 모든 진료과 방문 완료.")
            return Status.FAILURE # 큐가 비었으므로 실패 반환 -> 루프 종료 트리거

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
    """
    자식 노드를 실행하고, 자식이 FAILURE를 반환할 때까지 
    계속 RUNNING 상태를 유지하며 반복 실행합니다.
    """
    # [수정된 부분] 초기화 함수(__init__) 추가
    def __init__(self, name, children=None):
        super().__init__(name)
        # XML 파서가 넘겨준 자식 노드들을 내 리스트에 저장
        self.children = children if children is not None else []

    def _tick(self, agent, blackboard):
        # 자식이 없으면 에러(실패) 처리
        if not self.children:
            return Status.FAILURE
            
        # 자식 노드 실행 (Decorator이므로 첫 번째 자식만 실행)
        status = self.children[0]._tick(agent, blackboard)
        
        # 자식이 실패하면 -> 나도 실패(루프 종료)
        if status == Status.FAILURE:
            return Status.FAILURE
            
        # 자식이 성공하거나 실행 중이면 -> 나는 계속 실행 중(RUNNING)
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