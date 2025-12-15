import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StartLimoNode(Node):
    def __init__(self):
        super().__init__('start_limo_node')
        self.sub = self.create_subscription(
            String,
            '/form_submitted',
            self.cb,
            10
        )
        self.get_logger().info('🟢 StartLimoNode ready')

    def cb(self, msg):
        self.get_logger().info('🚀 출발 신호 수신')
        # TODO: Nav2 goal 보내기


def main():
    rclpy.init()
    node = StartLimoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

