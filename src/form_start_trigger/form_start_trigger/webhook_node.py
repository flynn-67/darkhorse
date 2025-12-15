# webhook_node.py
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request

app = Flask(__name__)

TOPIC_NAME = '/hospital/system_start'   # ✅ 여기로 통일 (오타 토픽 제거)

class FormWebhookNode(Node):
    def __init__(self):
        super().__init__('form_webhook_node')

        self.pub = self.create_publisher(String, TOPIC_NAME, 10)
        self.get_logger().info(f"Publisher ready: {TOPIC_NAME}")

        @app.route('/form', methods=['POST'])
        def form_webhook():
            data = request.get_json(force=True, silent=True)
            self.get_logger().info(f'RAW JSON: {data}')

            if not data or 'patient_id' not in data:
                self.get_logger().error('❌ patient_id 없음')
                return 'Bad Request', 400

            patient_id = data['patient_id']

            msg = String()
            # 1) 기존 방식: 단순 key=value 문자열
            msg.data = f'patient_id={patient_id}'

            # 2) (추천) JSON으로 보내고 싶으면 위 줄 대신 아래로:
            # import json
            # msg.data = json.dumps({"patient_id": patient_id}, ensure_ascii=False)

            self.pub.publish(msg)

            self.get_logger().info(
                f'📨 Webhook → {TOPIC_NAME} 발행: patient_id={patient_id}'
            )
            return 'OK', 200


def main():
    rclpy.init()
    node = FormWebhookNode()

    threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False
        ),
        daemon=True
    ).start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
