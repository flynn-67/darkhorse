#!/usr/bin/env python3
import threading
import json
from flask import Flask, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ==========================================
# 웹훅 서버 초기화
# ==========================================
app = Flask(__name__)

# JotForm 웹훅이 POST로 보내는 JSON 예시:
# {
#   "name": "홍길동",
#   "submission_id": "123456789012345"
# }

# **************************************
#   웹훅 요청 처리
# **************************************
@app.route('/patient_webhook', methods=['POST'])
def webhook():
    data = request.json

    name = data.get("name", "Unknown")
    submission_id = data.get("submission_id", "000")

    qr_url = f"https://quickchart.io/qr?text=https://form.jotform.com/253293055163051?patientId={submission_id}"

    # ROS 퍼블리시 (전역 변수 node 이용)
    msg = {
        "name": name,
        "patient_id": submission_id,
        "qr_url": qr_url
    }

    print(f"[Webhook 수신] {msg}")

    if api_node_ref is not None:
        api_node_ref.publish_data(json.dumps(msg))

    return {"status": "ok", "received": msg}, 200



# ==========================================
# ROS2 노드 클래스
# ==========================================
class QRAPINode(Node):
    def __init__(self):
        super().__init__('qr_api_node')

        self.pub_qr = self.create_publisher(String, '/scanned_qr_data', 10)
        self.get_logger().info("QR API Node 초기화 완료")

    def publish_data(self, json_data):
        msg = String()
        msg.data = json_data
        self.pub_qr.publish(msg)
        self.get_logger().info(f"ROS 송신: /scanned_qr_data → {json_data}")


# 전역 참조 변수
api_node_ref = None


# ==========================================
# Flask 서버를 스레드에서 실행
# ==========================================
def run_flask():
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)


# ==========================================
# 메인 실행
# ==========================================
def main():
    global api_node_ref

    rclpy.init()
    node = QRAPINode()
    api_node_ref = node

    # Flask 서버 실행 (서브스레드)
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    node.get_logger().info("Flask 웹서버 실행됨: http://localhost:5001/patient_webhook")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

