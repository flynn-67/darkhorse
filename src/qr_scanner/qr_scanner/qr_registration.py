import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import json

class QRRegistrationNode(Node):
    def __init__(self):
        super().__init__('qr_registration_node')
        
        # 1. 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # LIMO 실제 토픽명 확인 필요
            self.listener_callback,
            10)
        
        # 2. 시스템 전체에 "환자 확인됨" 신호 보내기
        # 민석(스마트폰 UI), 승훈(TTS) 등이 이 토픽을 구독함
        self.publisher_ = self.create_publisher(String, '/hospital/system_start', 10)
        
        self.bridge = CvBridge()
        self.waiting_number = 1  # 대기번호 카운터
        self.is_processing = False # 중복 인식 방지

    def listener_callback(self, msg):
        if self.is_processing: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            decoded_objects = decode(cv_image)

            for obj in decoded_objects:
                # 1. JotForm에서 만든 QR 내용을 읽음 (예: "589201923...")
                qr_data = obj.data.decode('utf-8')
                self.get_logger().info(f"JotForm QR 인식됨: {qr_data}")
                
                # 2. 팀원들에게 보낼 메시지 (JSON 형식으로 포장)
                # 민석(스마트폰 UI), 소진(대시보드)에게 "이 ID 환자 받았다"고 알림
                system_msg = {
                    "type": "qr_login",
                    "patient_id": qr_data,  # JotForm ID 그대로 전송
                    "waiting_number": self.waiting_number
                }
                
                # JSON으로 변환해서 퍼블리시
                self.publisher_.publish(String(data=json.dumps(system_msg)))
                
                self.waiting_number += 1
                self.is_processing = True

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = QRRegistrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()