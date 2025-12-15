import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_DEPTS = ["진단검사의학과", "영상의학과", "내과", "정형외과", "신경과"]

class HospitalSetupNode(Node):
    def __init__(self):
        super().__init__("hospital_setup_node")
        self.pub = self.create_publisher(String, "/hospital/departments_config", 10)

        # 1초마다 계속 뿌려서(라칭처럼) 늦게 켜진 노드도 받을 수 있게
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info("hospital_setup running: publishing /hospital/departments_config")

    def _tick(self):
        msg = String()
        msg.data = json.dumps({"departments": DEFAULT_DEPTS}, ensure_ascii=False)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HospitalSetupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
