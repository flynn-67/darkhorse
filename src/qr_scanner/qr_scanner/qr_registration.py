
import json
import time
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from pyzbar.pyzbar import decode


class QRRegistrationNode(Node):
    """
    - camera 이미지에서 QR 읽음
    - QR 내용(= submissionID 숫자)을 /hospital/qr_login 으로 JSON publish
    - 중복 인식 방지: cooldown + 최근 ID 캐시
    """

    def __init__(self):
        super().__init__("qr_registration_node")

        # params
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("publish_topic", "/hospital/qr_login")
        self.declare_parameter("cooldown_sec", 2.0)   # 같은 QR 연속 인식 방지
        self.declare_parameter("cache_size", 20)      # 최근 ID 기억(재인식 방지)

        self.image_topic = self.get_parameter("image_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.cache_size = int(self.get_parameter("cache_size").value)

        # ros
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.publish_topic, 10)

        self.bridge = CvBridge()

        self.waiting_number = 1
        self.last_pub_time = 0.0
        self.recent_ids = deque(maxlen=self.cache_size)

        self.get_logger().info(f"[QR] subscribe: {self.image_topic}")
        self.get_logger().info(f"[QR] publish : {self.publish_topic}")

    def on_image(self, msg: Image):
        now = time.time()
        if (now - self.last_pub_time) < self.cooldown_sec:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[QR] cv_bridge error: {e}")
            return

        decoded = decode(frame)
        if not decoded:
            return

        # 첫 QR만 처리
        qr_raw = decoded[0].data.decode("utf-8", errors="ignore").strip()
        if not qr_raw:
            return

        # 혹시 URL로 들어오면 숫자만 최대한 뽑아줌(안전장치)
        qr_id = self._extract_submission_id(qr_raw)
        if not qr_id:
            self.get_logger().warn(f"[QR] invalid data: {qr_raw}")
            return

        if qr_id in self.recent_ids:
            # 최근에 처리한 ID면 무시
            return

        payload = {
            "type": "qr_login",
            "patient_id": qr_id,                  # 제출ID(숫자)
            "waiting_number": self.waiting_number
        }

        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f"[QR] publish: {payload}")

        self.recent_ids.append(qr_id)
        self.waiting_number += 1
        self.last_pub_time = now

    def _extract_submission_id(self, s: str) -> str:
        """
        QR에 숫자만 넣는 게 정석.
        그래도 실수로 URL 들어왔을 때, 숫자 ID만 뽑아내는 안전장치.
        """
        # 1) 숫자만이면 그대로
        if s.isdigit():
            return s

        # 2) 문자열에서 "연속 숫자 덩어리" 후보 중 가장 긴 것 선택
        digits = []
        cur = []
        for ch in s:
            if ch.isdigit():
                cur.append(ch)
            else:
                if cur:
                    digits.append("".join(cur))
                    cur = []
        if cur:
            digits.append("".join(cur))

        if not digits:
            return ""

        # submissionID가 보통 길어서(예: 16~20자리) 가장 긴 덩어리를 선택
        digits.sort(key=len, reverse=True)
        return digits[0]


def main(args=None):
    rclpy.init(args=args)
    node = QRRegistrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
