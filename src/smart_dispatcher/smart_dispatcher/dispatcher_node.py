import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import random
import time
import json
import os
import sys

class DeptDispatcher(Node):
    def __init__(self):
        super().__init__('dept_dispatcher')
        
        # 1. 마스터 좌표 데이터베이스 (엔지니어가 미리 측정한 좌표값)
        # 요청하신 5개 과로 이름 변경 완료
        self.master_coordinates = {
            "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
            "영상의학과":    {"x": 6.578537940979004, "y": 2.621462106704712, "w": 1.0},
            "내과":          {"x": 7.445363998413086, "y": 0.5102964639663696, "w": 1.0},
            "정형외과":      {"x": 0.753912627696991, "y": -2.640972375869751, "w": 1.0},
            "신경과":        {"x": 2.836460590362549, "y": 1.1752597093582153, "w": 1.0}
        }

        # 2. 설정 파일 로드 (병원에서 선택한 과만 활성화)
        self.active_departments = self.load_config()

    def load_config(self):
        """저장된 설정 파일을 읽어서 활성화할 과 리스트를 반환"""
        config_path = os.path.expanduser('~/hospital_config.json')
        
        if not os.path.exists(config_path):
            self.get_logger().error(f"설정 파일이 없습니다! ({config_path})")
            self.get_logger().error("먼저 'ros2 run hospital_setup configure'를 실행하여 병원을 세팅해주세요.")
            sys.exit(1) # 강제 종료

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                selected = data.get("active_departments", [])
                
                # 좌표 데이터에 있는 것만 필터링 (안전장치)
                valid_depts = [d for d in selected if d in self.master_coordinates]
                
                print(f"📂 병원 설정 로드 완료: {valid_depts}")
                return valid_depts
        except Exception as e:
            self.get_logger().error(f"설정 파일 읽기 실패: {e}")
            sys.exit(1)

    def get_status_and_target(self):
        """활성화된 과 중에서만 대기 인원을 체크하고 목적지를 결정"""
        waiting_counts = {}
        print("\n--- [실시간 대기 인원 현황] ---")
        
        # 설정된 과들만 순회
        for dept in self.active_departments:
            count = random.randint(0, 10) # 랜덤 시뮬레이션
            waiting_counts[dept] = count
            print(f"{dept}: {count}명 대기 중")
            
        target_dept = min(waiting_counts, key=waiting_counts.get)
        min_count = waiting_counts[target_dept]
        
        print(f"-----------------------------")
        print(f"👉 추천 이동 장소: [{target_dept}] (대기: {min_count}명)")
        
        return target_dept

def main():
    rclpy.init()
    navigator = BasicNavigator()
    dispatcher = DeptDispatcher() # 초기화 시 설정 파일 로드됨

    navigator.waitUntilNav2Active()

    while rclpy.ok():
        target_name = dispatcher.get_status_and_target()
        target_info = dispatcher.master_coordinates[target_name]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_info['x']
        goal_pose.pose.position.y = target_info['y']
        goal_pose.pose.orientation.w = target_info['w']

        print(f"🚀 [{target_name}]로 이동 시작...")
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"✅ [{target_name}] 도착 완료! 업무 수행 중...")
            time.sleep(3.0)
        
        # (생략: 실패/취소 처리는 이전 코드와 동일)
        
        print("🔄 다음 경로 탐색 중...\n")

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()