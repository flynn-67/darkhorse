import tkinter as tk
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class PatientUI(Node):
    def __init__(self):
        super().__init__('patient_ui')

        # -------- Publish (제어) --------
        self.pub_speed = self.create_publisher(Float32, '/nav_speed_delta', 10)
        self.pub_pause = self.create_publisher(Bool, '/nav_pause', 10)
        self.pub_emg   = self.create_publisher(Bool, '/nav_emergency_home', 10)
        self.pub_next  = self.create_publisher(Bool, '/hospital/next_waypoint', 10)

        # -------- Subscribe (표시: 속도만) --------
        self.speed = 0.0
        self.create_subscription(Float32, '/nav_current_speed', self.cb_speed, 10)

        self.paused = False

    def cb_speed(self, msg: Float32):
        self.speed = float(msg.data)


class App:
    def __init__(self, node: PatientUI):
        self.node = node

        self.root = tk.Tk()
        self.root.title("환자 안내 화면")
        self.root.geometry("480x420")
        self.root.configure(bg="#F6F7FB")

        # Enter → 다음 waypoint
        self.root.bind("<Return>", self.on_enter)

        # -------- 속도 표시 --------
        card = tk.Frame(self.root, bg="white",
                        highlightthickness=1, highlightbackground="#E5E7EB")
        card.pack(padx=20, pady=30, fill="x")

        tk.Label(
            card,
            text="현재 이동 속도",
            font=("Arial", 14),
            bg="white"
        ).pack(pady=(16, 6))

        self.lbl_speed = tk.Label(
            card,
            text="0.00 m/s",
            font=("Arial", 28, "bold"),
            bg="white"
        )
        self.lbl_speed.pack(pady=(0, 16))

        # -------- 속도 조절 --------
        row = tk.Frame(self.root, bg="#F6F7FB")
        row.pack(padx=20, pady=10, fill="x")

        tk.Button(
            row, text="속도 -", font=("Arial", 16, "bold"),
            height=2,
            command=lambda: self.node.pub_speed.publish(Float32(data=-0.05))
        ).pack(side="left", expand=True, fill="x", padx=6)

        tk.Button(
            row, text="속도 +", font=("Arial", 16, "bold"),
            height=2,
            command=lambda: self.node.pub_speed.publish(Float32(data=+0.05))
        ).pack(side="left", expand=True, fill="x", padx=6)

        # -------- 정지 / 재개 --------
        self.btn_pause = tk.Button(
            self.root, text="정지",
            font=("Arial", 16, "bold"),
            height=2,
            bg="#111827", fg="white",
            command=self.toggle_pause
        )
        self.btn_pause.pack(padx=20, pady=(16, 10), fill="x")

        # -------- Emergency --------
        tk.Button(
            self.root, text="EMERGENCY (HOME 복귀)",
            font=("Arial", 15, "bold"),
            height=2,
            bg="#DC2626", fg="white",
            command=lambda: self.node.pub_emg.publish(Bool(data=True))
        ).pack(padx=20, pady=(6, 20), fill="x")

        self.update_ui()

    def on_enter(self, event=None):
        self.node.pub_next.publish(Bool(data=True))

    def toggle_pause(self):
        self.node.paused = not self.node.paused
        self.node.pub_pause.publish(Bool(data=self.node.paused))
        self.btn_pause.config(text="재개" if self.node.paused else "정지")

    def update_ui(self):
        self.lbl_speed.config(text=f"{self.node.speed:.2f} m/s")
        self.root.after(200, self.update_ui)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = PatientUI()

    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    App(node).run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
