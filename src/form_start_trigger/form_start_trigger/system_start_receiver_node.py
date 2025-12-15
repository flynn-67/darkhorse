import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math

class SystemStartReceiverNode(Node):
    def __init__(self):
        super().__init__('system_start_receiver_node')

        self.started = False

        # system_start 토픽 구독
        self.sub = self.create_subscription(
            String,
            '/hospital/system_start',
            self.system_start_cb,
            10
        )

        # Nav2 Action Client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info('🟢 system_start 수신 대기 중')

    def system_start_cb(self, msg: String):
        if self.started:
            self.get_logger().warn('⚠️ 이미 출발 처리됨, 무시')
            return

        self.started = True
        self.get_logger().info(f'🚀 system_start 수신: {msg.data}')

        # patient_id 파싱
        patient_id = self.parse_patient_id(msg.data)
        self.get_logger().info(f'🧾 patient_id = {patient_id}')

        # waypoint 선택 (예시)
        goal_pose = self.make_waypoint(patient_id)

        # Nav2 goal 전송
        self.send_nav_goal(goal_pose)

    def parse_patient_id(self, data: str):
        # 예: "patient_id=17"
        try:
            return int(data.split('=')[1])
        except Exception:
            return -1

    def make_waypoint(self, patient_id: int):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 🔽 예시 waypoint (네 맵에 맞게 수정)
        # patient_id에 따라 분기 가능
        if patient_id % 2 == 0:
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.0
        else:
            pose.pose.position.x = 0.0
            pose.pose.position.y = 1.0

        yaw = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_nav_goal(self, pose: PoseStamped):
        self.get_logger().info('📡 Nav2 서버 대기 중...')
        self.nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info('🧭 Nav2 goal 전송')
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 goal 거절됨')
            return

        self.get_logger().info('✅ Nav2 goal 수락됨')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.get_logger().info('🎯 목적지 도착 완료')

    def feedback_cb(self, feedback_msg):
        # 필요하면 진행 상황 로그
        pass


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

