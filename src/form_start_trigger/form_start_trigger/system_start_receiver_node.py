#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import json
import math

class SystemStartReceiverNode(Node):
    def __init__(self):
        super().__init__('system_start_receiver_node')
        self.started = False

        # ✅ 폼 제출 데이터 수신
        self.create_subscription(
            String,
            '/hospital/mission_data',
            self.cb_mission_data,
            10
        )

        # 미션 완료 신호 발행
        self.pub_mission_completed = self.create_publisher(
            Bool,
            '/hospital/mission_completed',
            10
        )

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info('🟢 mission_data 대기 중 (Simple Controller)')

    def cb_mission_data(self, msg: String):
        if self.started:
            return

        self.started = True
        try:
            data = json.loads(msg.data)
            patient_id = data.get("patient_id")
            self.get_logger().info(f'🚀 미션 시작: patient_id={patient_id}')
            
            goal = self.make_waypoint(patient_id)
            self.send_nav_goal(goal)
        except Exception as e:
            self.get_logger().error(f"데이터 파싱 실패: {e}")
            self.started = False

    def make_waypoint(self, patient_id):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 예시 분기 (테스트용 하드코딩 좌표)
        if str(patient_id).endswith("0"):
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.0
        else:
            pose.pose.position.x = 0.0
            pose.pose.position.y = 1.0

        yaw = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_nav_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server 응답 없음")
            self.started = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info("Nav2 목표 전송 중...")
        self.nav_client.send_goal_async(
            goal
        ).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 goal 거절')
            self.started = False
            return

        self.get_logger().info("Nav2 목표 수락됨. 이동 시작.")
        goal_handle.get_result_async().add_done_callback(
            self.result_cb
        )

    def result_cb(self, future):
        self.get_logger().info('🎯 목적지 도착')
        
        # 완료 신호 발행
        msg = Bool()
        msg.data = True
        self.pub_mission_completed.publish(msg)
        
        # 다음 요청을 위해 플래그 초기화 (원하면 유지)
        self.started = False 

def main():
    rclpy.init()
    node = SystemStartReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
