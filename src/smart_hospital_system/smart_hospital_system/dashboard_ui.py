#!/usr/bin/env python3
import faulthandler
faulthandler.enable()

import os
import time
import threading
import queue
import json
import requests
import tkinter as tk

os.environ["GDK_BACKEND"] = "x11"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ==========================================
# 설정
# ==========================================
API_KEY = "e57cc1f435fe873f0fdf8ada20298ba1"
DOCTOR_FORM_ID = "253293055163051"  

FIELD_PATIENT_ID = "3"
FIELD_DIAGNOSIS = "4"
FIELD_DEPT = "5"


# ==========================================
# Dashboard UI Class
# ==========================================
class DoctorDashboard:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("의료진 통합 대시보드")
        self.root.geometry("420x650")
        self.root.configure(bg="white")

        self.running = True
        self.last_submission_id = None
        self.event_queue = queue.Queue()

        # ROS publishers
        self.pub_record = self.node.create_publisher(String, '/medical_record', 10)
        self.pub_finish = self.node.create_publisher(Bool, '/exam_finished', 10)

        # QR API Subscriber 추가
        self.sub_api_qr = self.node.create_subscription(
            String,
            '/scanned_qr_data',
            lambda msg: self.event_queue.put(("qr_api", msg.data)),
            10
        )

        # UI 구성
        self.create_ui()

        # JotForm 감시 스레드(보조 기능)
        self.monitor_thread = threading.Thread(target=self.loop_check_jotform, daemon=True)
        self.monitor_thread.start()

        # UI 업데이트 루프
        self.root.after(100, self.update_loop)

        print(">>> [Dashboard] 실행됨 (QR API + JotForm 감시 중)")

    # ==========================================
    # UI 구성
    # ==========================================
    def create_ui(self):
        tk.Label(self.root, text="의료진 대시보드", 
                 font=("Helvetica", 18, "bold"), bg="white").pack(pady=15)

        self.lbl_status = tk.Label(
            self.root, text="대기 중 ...",
            font=("Helvetica", 14), fg="blue", bg="white"
        )
        self.lbl_status.pack(pady=5)

        self.txt_log = tk.Text(self.root, height=25, width=45, font=("Helvetica", 10))
        self.txt_log.pack(padx=20, pady=20)
        self.txt_log.insert(tk.END, "[로그 시작]\n시스템이 QR 및 JotForm 데이터를 기다리는 중...\n\n")

    # ==========================================
    # UI 업데이트 루프 (메인 스레드)
    # ==========================================
    def update_loop(self):
        try:
            while True:
                msg_type, data = self.event_queue.get_nowait()

                if msg_type == "qr_api":
                    self.process_qr_api_data(data)

                elif msg_type == "jotform":
                    self.process_submission(data)

        except queue.Empty:
            pass

        self.root.after(100, self.update_loop)

    # ==========================================
    # 1) QR API 서버 데이터 처리
    # ==========================================
    def process_qr_api_data(self, json_data):
        try:
            info = json.loads(json_data)
            p_id = info.get("patient_id", "Unknown")
            name = info.get("name", "Unknown")

            log_msg = (
                f"\n[QR 스캔 감지]\n"
                f"환자 이름: {name}\n"
                f"환자 ID: {p_id}\n"
            )

            self.txt_log.insert(tk.END, log_msg)
            self.txt_log.see(tk.END)
            self.lbl_status.config(text=f"QR 스캔 감지됨 → ID {p_id}")

        except Exception as e:
            print(f"[QR API 처리 오류] {e}")

    # ==========================================
    # 2) JotForm 제출 확인 (보조 기능)
    # ==========================================
    def loop_check_jotform(self):
        print(">>> JotForm 감시 스레드 시작")

        while self.running:
            try:
                url = f"https://api.jotform.com/form/{DOCTOR_FORM_ID}/submissions?apiKey={API_KEY}&limit=1&orderby=created_at"
                response = requests.get(url, timeout=5)

                if response.status_code == 200:
                    data = response.json().get("content", [])
                    if data:
                        submission = data[0]
                        sub_id = submission.get("id")

                        if sub_id != self.last_submission_id:
                            self.last_submission_id = sub_id
                            self.event_queue.put(("jotform", submission))

                elif response.status_code == 429:
                    print("[대시보드] 429 Too Many Requests → 잠시 대기")

            except Exception as e:
                print(f"[API Error] {e}")

            time.sleep(5)

    # ==========================================
    # JotForm 제출 처리 → 로봇/키오스크 전달
    # ==========================================
    def process_submission(self, sub):
        answers = sub.get("answers", {})
        
        p_id = answers.get(FIELD_PATIENT_ID, {}).get("answer", "Unknown")
        diagnosis = answers.get(FIELD_DIAGNOSIS, {}).get("answer", "소견 없음")
        dept = answers.get(FIELD_DEPT, {}).get("answer", "일반의")

        log_msg = (
            "------------------------\n"
            f"[JotForm 제출 수신]\n"
            f"환자 ID: {p_id}\n"
            f"진료과: {dept}\n"
            f"소견: {diagnosis}\n"
        )

        self.txt_log.insert(tk.END, log_msg)
        self.txt_log.see(tk.END)
        self.lbl_status.config(text=f"JotForm 제출 수신 → {p_id}")

        # ROS 퍼블리시
        record_data = {
            "id": p_id,
            "dept": dept,
            "diagnosis": diagnosis,
            "timestamp": time.time()
        }

        self.pub_record.publish(String(data=json.dumps(record_data)))
        self.pub_finish.publish(Bool(data=True))

        print(f"[ROS] /medical_record 발행 완료 ({p_id})")
        print("[ROS] /exam_finished = True")

# ==========================================
# ROS 스레드
# ==========================================
def ros_spin(node):
    rclpy.spin(node)

# ==========================================
# 메인 실행
# ==========================================
def main():
    rclpy.init()
    node = Node('dashboard_ui_node')

    root = tk.Tk()
    app = DoctorDashboard(root, node)

    t = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    t.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.running = False
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

