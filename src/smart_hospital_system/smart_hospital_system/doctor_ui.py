#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox, scrolledtext
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import threading
import queue
import requests
import time

# ==========================================
# [설정] 의사용 JotForm 정보
# ==========================================
API_KEY = "e57cc1f435fe873f0fdf8ada20298ba1"
DOCTOR_FORM_ID = "253293055163051"

FIELD_ID_PATIENT_NUM = "3"
FIELD_ID_OPINION = "4"


# ==========================================
# Doctor UI Class
# ==========================================
class DoctorDashboard:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("👨‍⚕️ 의료진 통합 대시보드")
        self.root.geometry("600x600")

        self.current_patient_id = None
        self.last_submission_id = None
        self.event_queue = queue.Queue()
        self.running = True

        self.setup_ui()

        # --------------------------
        # ROS Publishers
        # --------------------------
        self.pub_record = self.node.create_publisher(String, '/medical_record', 10)
        self.pub_finish = self.node.create_publisher(Bool, '/exam_finished', 10)

        # --------------------------
        # QR API 서버에서 들어오는 데이터 구독
        # --------------------------
        self.sub_api_qr = self.node.create_subscription(
            String,
            '/scanned_qr_data',
            lambda msg: self.event_queue.put(("qr_api", msg.data)),
            10
        )

        # --------------------------
        # JotForm 감시 스레드 (보조 기능)
        # --------------------------
        self.jotform_thread = threading.Thread(target=self.loop_check_jotform, daemon=True)
        self.jotform_thread.start()

        # UI 메인 루프
        self.root.after(100, self.update_loop)

        print(">>> [Doctor UI] 실행됨 (QR + JotForm 대기 중)")

    # ==========================================
    # UI 구성
    # ==========================================
    def setup_ui(self):
        self.lbl_status = tk.Label(self.root, text="QR 또는 모바일 소견서 대기 중...",
                                   font=("Arial", 16, "bold"), bg="lightgray", pady=10)
        self.lbl_status.pack(fill="x")

        frame_info = tk.Frame(self.root, pady=10)
        frame_info.pack()
        tk.Label(frame_info, text="환자 ID:", font=("Arial", 12)).pack(side="left")
        self.lbl_patient_id = tk.Label(frame_info, text="-",
                                       font=("Arial", 14, "bold"), fg="blue")
        self.lbl_patient_id.pack(side="left", padx=10)

        tk.Label(self.root, text="[의사 소견서 내용]",
                 font=("Arial", 12, "bold"), fg="#4f46e5").pack(pady=(20, 5))

        self.txt_diagnosis = scrolledtext.ScrolledText(
            self.root, height=12, width=55, font=("Arial", 11))
        self.txt_diagnosis.insert("1.0", "QR 스캔 또는 모바일 제출을 기다리는 중입니다.")
        self.txt_diagnosis.config(state="disabled", bg="#f0f4f8")
        self.txt_diagnosis.pack(padx=20)

        frame_btn = tk.Frame(self.root, pady=20)
        frame_btn.pack()
        self.btn_save = tk.Button(frame_btn, text="최종 확정 및 전송",
                                  bg="#00CC66", fg="white",
                                  font=("Arial", 12, "bold"),
                                  state="disabled", command=self.send_report)
        self.btn_save.pack()

    # ==========================================
    # UI 루프: 메시지 처리
    # ==========================================
    def update_loop(self):
        try:
            while True:
                msg_type, data = self.event_queue.get_nowait()

                # QR 스캔 데이터 처리
                if msg_type == "qr_api":
                    self.handle_qr_api_data(data)

                # 모바일 통한 JotForm 제출 처리
                elif msg_type == "jotform_data":
                    self.display_jotform_data(data)

        except queue.Empty:
            pass

        self.root.after(100, self.update_loop)

    # ==========================================
    # 1) QR API 데이터 처리 (가장 중요한 기능)
    # ==========================================
    def handle_qr_api_data(self, json_data):
        """
        QR API 서버에서 /scanned_qr_data로 들어온 JSON 처리
        {
            "name": "테스트",
            "patient_id": "555",
            "qr_url": "..."
        }
        """
        try:
            info = json.loads(json_data)
            patient_id = info.get("patient_id", "Unknown")

            print(f"\n[QR API] 스캔된 환자 ID: {patient_id}")

            # UI 업데이트
            self.current_patient_id = patient_id
            self.lbl_status.config(text="🔵 QR 스캔 감지됨!", bg="#DBEAFE", fg="#1E3A8A")
            self.lbl_patient_id.config(text=patient_id)

            # 소견 입력창 활성화
            self.txt_diagnosis.config(state="normal", bg="white")
            self.txt_diagnosis.delete("1.0", tk.END)
            self.txt_diagnosis.insert(tk.END, "환자 진료 소견을 입력하세요...\n")
            self.btn_save.config(state="normal")

        except Exception as e:
            print(f"[QR API 파싱 오류] {e}")

    # ==========================================
    # 2) 모바일 소견서 제출(JotForm) 처리
    # ==========================================
    def loop_check_jotform(self):
        while self.running:
            try:
                url = f"https://api.jotform.com/form/{DOCTOR_FORM_ID}/submissions?apiKey={API_KEY}&limit=1&orderby=created_at"
                response = requests.get(url, timeout=5)

                if response.status_code == 200:
                    content = response.json().get("content", [])
                    if content:
                        sub = content[0]
                        sub_id = sub.get("id")

                        if sub_id != self.last_submission_id:
                            self.last_submission_id = sub_id
                            self.event_queue.put(("jotform_data", sub))

                elif response.status_code == 429:
                    print("[JotForm] 429 Too Many Requests → 대기")

            except Exception as e:
                print(f"[API 오류] {e}")

            time.sleep(5)

    # ==========================================
    # 폼 제출로 받은 소견서 표시
    # ==========================================
    def display_jotform_data(self, submission):
        answers = submission.get("answers", {})
        patient_id = answers.get(FIELD_ID_PATIENT_NUM, {}).get("answer", "Unknown")
        opinion = answers.get(FIELD_ID_OPINION, {}).get("answer", "")

        print(f"[JotForm] 제출된 소견서 도착! 환자ID:{patient_id}")

        self.current_patient_id = patient_id

        self.lbl_status.config(text="🟢 모바일 소견서 도착!", bg="#D1FAE5", fg="#065F46")
        self.lbl_patient_id.config(text=patient_id)

        self.txt_diagnosis.config(state="normal", bg="white")
        self.txt_diagnosis.delete("1.0", tk.END)
        self.txt_diagnosis.insert(tk.END, opinion)

        self.btn_save.config(state="normal")

    # ==========================================
    # 소견서 확정 → ROS 전달
    # ==========================================
    def send_report(self):
        diagnosis = self.txt_diagnosis.get("1.0", tk.END).strip()
        if not diagnosis:
            messagebox.showwarning("경고", "내용이 비어있습니다.")
            return

        record = {
            "id": self.current_patient_id,
            "dept": "내과",
            "diagnosis": diagnosis
        }

        self.pub_record.publish(String(data=json.dumps(record)))
        self.pub_finish.publish(Bool(data=True))

        messagebox.showinfo("전송 완료", "환자에게 최종 소견서를 전송했습니다.")

        # UI 초기화
        self.lbl_status.config(text="다음 QR 또는 모바일 소견서 대기 중...",
                               bg="lightgray", fg="black")
        self.lbl_patient_id.config(text="-")

        self.txt_diagnosis.config(state="disabled", bg="#f0f4f8")
        self.txt_diagnosis.delete("1.0", tk.END)
        self.txt_diagnosis.insert("1.0", "QR 또는 JotForm 제출을 기다리는 중입니다.")

        self.btn_save.config(state="disabled")
        self.current_patient_id = None


# ==========================================
# ROS 스레드
# ==========================================
def ros_thread(node):
    rclpy.spin(node)


# ==========================================
# 메인 실행
# ==========================================
def main():
    if not rclpy.ok():
        rclpy.init()

    node = Node('doctor_dashboard_node')
    root = tk.Tk()
    app = DoctorDashboard(root, node)

    t = threading.Thread(target=ros_thread, args=(node,), daemon=True)
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

