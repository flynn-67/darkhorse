#!/usr/bin/env python3
import faulthandler
faulthandler.enable()  # 세그폴트 발생 시 원인 추적

import os
import time
import threading
import queue
import json
import random
import requests
from io import BytesIO

# [중요] tkinter를 rclpy보다 먼저 import 해야 충돌이 적습니다.
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import qrcode

# ROS 관련 import
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


# ==========================================
# 설정값
# ==========================================
API_KEY = "e57cc1f435fe873f0fdf8ada20298ba1"
FORM_ID = "253292147686062"       # 환자용 문진표
DOCTOR_FORM_ID = "253293055163051" # 의사용 문진표
FIELD_ID_NAME = "3"
FIELD_ID_UNIQUE_NUM = "12"
TARGET_FIELD_NAME = "input_3"

DEPARTMENTS = [
    {"name": "내과", "desc": "혈압 및 기본 검사"},
    {"name": "외과", "desc": "신체 외상 검사"},
    {"name": "이비인후과", "desc": "호흡기 정밀 검사"},
    {"name": "치과", "desc": "구강 건강 검진"}
]


# ==========================================
# UI 메인 클래스
# ==========================================
class SmartHospitalApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("스마트 병원 환자용 키오스크")
        self.root.geometry("500x850")
        self.root.configure(bg="#f0f4f8")

        # 상태 값
        self.patient_name = ""
        self.unique_id = None
        self.report_link = ""
        self.qr_image = None
        self.medical_records = []
        self.waiting_counts = {}
        self.dept_labels = {}
        self.last_submission_id = None
        self.running = True

        # 메시지 큐
        self.event_queue = queue.Queue()

        # UI 구성
        self.main_frame = tk.Frame(root, bg="#f0f4f8")
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        self.show_home_screen()

        # -------------------------------
        # ROS Publishers
        # -------------------------------
        self.pub_start = self.node.create_publisher(String, '/hospital_data', 10)
        self.pub_emergency = self.node.create_publisher(Bool, '/emergency', 10)

        # -------------------------------
        # ROS Subscribers
        # -------------------------------
        self.sub_loc = self.node.create_subscription(String, '/current_hospital',
                lambda m: self.event_queue.put(("location", m.data)), 10)
        self.sub_fin = self.node.create_subscription(Bool, '/exam_finished',
                lambda m: self.event_queue.put(("finish", m.data)), 10)
        self.sub_record = self.node.create_subscription(String, '/medical_record',
                lambda m: self.event_queue.put(("record", m.data)), 10)

        # [중요] QR API 서버에서 퍼블리시되는 토픽 구독
        self.sub_api_qr = self.node.create_subscription(String, '/scanned_qr_data',
                lambda m: self.event_queue.put(("scanner_api", m.data)), 10)

        # -------------------------------
        # UI 업데이트 루프 실행
        # -------------------------------
        self.root.after(100, self.update_ui_loop)

        # JotForm 폴링 스레드 시작
        self.jotform_thread = threading.Thread(target=self.loop_check_jotform, daemon=True)
        self.jotform_thread.start()

        print(">>> [UI] 키오스크 실행됨")


    # ==========================================
    # UI 업데이트 루프
    # ==========================================
    def update_ui_loop(self):
        if not self.running:
            return
        
        try:
            while True:
                msg_type, data = self.event_queue.get_nowait()

                if msg_type == "jotform":
                    self.process_new_submission(data)

                elif msg_type == "location":
                    name = data.strip()
                    if name in self.dept_labels:
                        self.dept_labels[name].config(text="진료 중", fg="#4f46e5")

                elif msg_type == "record":
                    try:
                        r = json.loads(data)
                        self.medical_records.append(r)
                        dept = r.get('dept')
                        if dept in self.dept_labels:
                            self.dept_labels[dept].config(text="완료", fg="#10b981")
                    except:
                        pass

                elif msg_type == "finish":
                    if data:
                        self.show_final_report()

                # 여기서 API QR 데이터를 처리
                elif msg_type == "scanner_api":
                    self.handle_api_qr_data(data)

        except queue.Empty:
            pass

        # 반복 실행
        self.root.after(100, self.update_ui_loop)


    # ==========================================
    # JotForm 자동 체크
    # ==========================================
    def loop_check_jotform(self):
        while self.running:
            try:
                url = f"https://api.jotform.com/form/{FORM_ID}/submissions?apiKey={API_KEY}&limit=1&orderby=created_at"
                response = requests.get(url, timeout=5)

                if response.status_code == 200:
                    content = response.json().get("content", [])
                    if content:
                        sub = content[0]
                        sub_id = sub.get("id")

                        if sub_id != self.last_submission_id:
                            self.last_submission_id = sub_id
                            self.event_queue.put(("jotform", sub))
                            print(f"[JotForm] 데이터 수신 성공! ID: {sub_id}")

                elif response.status_code == 429:
                    print("[오류] 429 Too Many Requests → 잠시 대기")
                    time.sleep(10)

            except Exception as e:
                print(f"[통신 오류] {e}")

            time.sleep(5)


    # ==========================================
    # 환자 모바일 제출 처리 → QR 생성
    # ==========================================
    def process_new_submission(self, submission):
        answers = submission.get("answers", {})

        raw_name = answers.get(FIELD_ID_NAME, {}).get("answer", "방문자")
        if isinstance(raw_name, dict):
            self.patient_name = f"{raw_name.get('last','')} {raw_name.get('first','')}"
        else:
            self.patient_name = raw_name

        self.unique_id = answers.get(FIELD_ID_UNIQUE_NUM, {}).get("answer", "000")

        # 의사용 링크 생성
        self.report_link = f"https://form.jotform.com/{DOCTOR_FORM_ID}?{TARGET_FIELD_NAME}={self.unique_id}"
        print(f"[처리 중] 환자: {self.patient_name}, ID: {self.unique_id}")

        # QR 생성
        try:
            qr = qrcode.QRCode(box_size=10, border=4)
            qr.add_data(self.report_link)
            qr.make(fit=True)
            pil_image = qr.make_image(fill_color="black", back_color="white").convert('RGB')
            pil_image = pil_image.resize((250, 250))
            self.qr_image = ImageTk.PhotoImage(pil_image)
        except Exception as e:
            print(f"QR 생성 실패: {e}")
            self.qr_image = None

        self.show_qr_simulation()


    # ==========================================
    # 새로 추가됨: QR API 서버 → 환자 UI 연결
    # ==========================================
    def handle_api_qr_data(self, json_data):
        """QR API 서버에서 수신한 데이터 처리"""
        try:
            info = json.loads(json_data)

            name = info.get("name", "Unknown")
            pid = info.get("patient_id", "000")
            url = info.get("qr_url", "")

            print(f"[API QR 수신] 이름:{name}, ID:{pid}, URL:{url}")

            # 원하는 UI 연동을 여기에 넣을 수 있음 (예: 화면에 환자 표시)
            # ex) self.lbl_status.config(text=f"QR 스캔됨! 환자ID {pid}")

        except Exception as e:
            print(f"[QR API 파싱 오류] {e}")


    # ==========================================
    # 기존 기능들 (변경 없음)
    # ==========================================
    def handle_scan(self, scanned_data):
        scanned_id = scanned_data.strip()
        print(f"[스캔] {scanned_id}")

    def clear_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.destroy()

    def show_home_screen(self):
        self.clear_frame()
        self.qr_image = None
        tk.Label(self.main_frame, text="스마트 병원", font=("Arial", 24, "bold"), fg="#4f46e5").pack(pady=40)
        tk.Label(self.main_frame, text="모바일 접수 대기 중...", font=("Arial", 16, "bold"), fg="#e11d48").pack(pady=10)
        tk.Button(self.main_frame, text="수동 접수", font=("Arial", 14),
                  command=self.show_questionnaire).pack(fill="x", pady=20)
        tk.Button(self.main_frame, text="긴급 호출", font=("Arial", 14, "bold"), bg="#ef4444", fg="white",
                  command=lambda: self.pub_emergency.publish(Bool(data=True))).pack(side="bottom", fill="x", pady=20)

    def show_questionnaire(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="이름 입력", font=("Arial", 18)).pack(pady=20)
        self.entry_name = tk.Entry(self.main_frame, font=("Arial", 12))
        self.entry_name.pack(fill="x", pady=5)
        tk.Button(self.main_frame, text="완료",
                  command=lambda: [
                      setattr(self, 'patient_name', self.entry_name.get()),
                      setattr(self, 'unique_id', '999'),
                      self.process_new_submission({'answers':{}})
                  ]).pack(fill="x", pady=20)

    def show_qr_simulation(self):
        self.clear_frame()
        tk.Label(self.main_frame, text=f"{self.patient_name}님 접수증", font=("Arial", 18)).pack(pady=20)

        if self.qr_image:
            tk.Label(self.main_frame, image=self.qr_image).pack(pady=20)
            tk.Label(self.main_frame, text=f"ID: {self.unique_id}", font=("Arial", 14, "bold")).pack()
            tk.Label(self.main_frame, text="의사 선생님이 이 QR을 스캔합니다", font=("Arial", 10), fg="gray").pack()
        else:
            tk.Label(self.main_frame, text="[QR 생성 실패]", bg="black", fg="white",
                     width=20, height=10).pack(pady=20)

        tk.Button(self.main_frame, text="로봇 연동 시작", bg="#00CC66", fg="white",
                  font=("Arial", 14), command=self.start_robot_system).pack(fill="x", pady=20)
        tk.Button(self.main_frame, text="처음으로",
                  command=self.show_home_screen).pack(fill="x", pady=10)

    def show_progress_view(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="대기 현황", font=("Arial", 18)).pack(pady=20)
        self.dept_labels = {}

        for dept in DEPARTMENTS:
            frame = tk.Frame(self.main_frame, bg="white", padx=10, pady=5)
            frame.pack(fill="x", pady=5)

            tk.Label(frame, text=dept['name'], width=10, anchor='w').pack(side="left")
            count = self.waiting_counts.get(dept['name'], 0)
            self.dept_labels[dept['name']] = tk.Label(frame, text=f"{count}명 대기", fg="red")
            self.dept_labels[dept['name']].pack(side="right")

    def show_final_report(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="모든 진료가 완료되었습니다!", font=("Arial", 20, "bold"),
                 fg="blue").pack(pady=50)
        tk.Button(self.main_frame, text="처음으로",
                  command=self.show_home_screen).pack(pady=20)

    def start_robot_system(self):
        self.waiting_counts = {d['name']: random.randint(1, 5) for d in DEPARTMENTS}
        self.pub_start.publish(String(data=json.dumps({
            'command': 'start',
            'patient_name': self.patient_name,
            **self.waiting_counts
        })))
        self.show_progress_view()


# ==========================================
# ROS 스레드 처리
# ==========================================
def ros_thread(node, app_ref):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok() and app_ref.running:
        executor.spin_once(timeout_sec=0.1)


# ==========================================
# 메인 실행
# ==========================================
def main():
    if "GDK_BACKEND" not in os.environ:
        os.environ["GDK_BACKEND"] = "x11"

    rclpy.init()
    node = Node('patient_ui_node')

    root = tk.Tk()
    app = SmartHospitalApp(root, node)

    t = threading.Thread(target=ros_thread, args=(node, app), daemon=True)
    t.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        print("[System] 종료 중...")
        app.running = False
        t.join(timeout=1.0)

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

        try:
            if root.winfo_exists():
                root.quit()
                root.destroy()
        except:
            pass


if __name__ == '__main__':
    main()

