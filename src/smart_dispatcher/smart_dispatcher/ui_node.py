import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool, String
import tkinter as tk
# font 모듈 import 제거 (안전성 확보)

class LimoControlUI(Node):
    def __init__(self, root):
        super().__init__('limo_control_ui')
        self.root = root
        self.root.title("LIMO Control")
        self.root.geometry("450x600")
        
        # 1. Nav2 클라이언트
        print("[DEBUG] Creating Clients...")
        self.client_controller = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.client_smoother = self.create_client(SetParameters, '/velocity_smoother/set_parameters')
        self.client_controller_get = self.create_client(GetParameters, '/controller_server/get_parameters')

        # 2. 통신 설정
        self.pub_emergency = self.create_publisher(Bool, '/emergency_trigger', 10)
        self.create_subscription(String, '/hospital/nav_status', self.update_status, 10)

        # 변수
        self.current_speed = 0.4
        self.min_speed = 0.1
        self.max_speed = 1.0
        self.is_emergency = False
        self.current_status_text = "IDLE"

        # 3. 위젯 생성
        print("[DEBUG] Creating Widgets...")
        self.create_widgets()
        
        # 4. ROS Loop
        self.root.after(100, self.ros_spin_once)
        self.sync_initial_speed()

    def ros_spin_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            pass
        self.root.after(100, self.ros_spin_once)

    def sync_initial_speed(self):
        if self.client_controller_get.service_is_ready():
            req = GetParameters.Request()
            req.names = ['FollowPath.max_vel_x']
            future = self.client_controller_get.call_async(req)
            future.add_done_callback(self._sync_callback)

    def _sync_callback(self, future):
        try:
            result = future.result()
            if result.values:
                val = result.values[0].double_value
                self.current_speed = round(val, 2)
                self.lbl_speed.config(text=f"Speed: {self.current_speed} m/s")
        except Exception: pass

    def create_widgets(self):
        # ⚠️ [안전 모드] 폰트, 색상 등 복잡한 옵션 제거
        
        # 상태 표시
        frame_status = tk.Frame(self.root, pady=10)
        frame_status.pack(fill="x")
        tk.Label(frame_status, text="Status:").pack()
        self.lbl_status = tk.Label(frame_status, text=self.current_status_text, font=("Arial", 16, "bold"))
        self.lbl_status.pack()

        # 속도 조절
        frame_speed = tk.LabelFrame(self.root, text="Speed Control", pady=10)
        frame_speed.pack(fill="x", padx=10, pady=10)
        
        self.lbl_speed = tk.Label(frame_speed, text=f"Speed: {self.current_speed} m/s", font=("Arial", 12))
        self.lbl_speed.pack()

        btn_frame = tk.Frame(frame_speed)
        btn_frame.pack(pady=5)
        
        tk.Button(btn_frame, text="<< Slower", width=10, command=lambda: self.change_speed(-0.1)).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Faster >>", width=10, command=lambda: self.change_speed(0.1)).pack(side="right", padx=5)

        # 비상 버튼
        frame_emg = tk.Frame(self.root, pady=20)
        frame_emg.pack(fill="both", expand=True, padx=10)
        
        self.btn_emergency = tk.Button(frame_emg, text="EMERGENCY STOP", bg="red", fg="white", 
                                       font=("Arial", 14, "bold"), command=self.toggle_emergency)
        self.btn_emergency.pack(fill="both", expand=True)

    def change_speed(self, delta):
        self.current_speed = round(max(self.min_speed, min(self.current_speed + delta, self.max_speed)), 2)
        self.lbl_speed.config(text=f"Speed: {self.current_speed} m/s")
        self._set_param(self.client_controller, 'FollowPath.max_vel_x', self.current_speed)
        self._set_param(self.client_smoother, 'max_velocity', [self.current_speed, 0.0, 1.0])

    def _set_param(self, client, name, val):
        if not client.service_is_ready(): return
        req = SetParameters.Request()
        p = Parameter()
        p.name = name
        if isinstance(val, list):
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(x) for x in val])
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
        req.parameters = [p]
        client.call_async(req)

    def toggle_emergency(self):
        self.is_emergency = not self.is_emergency
        msg = Bool()
        msg.data = self.is_emergency
        self.pub_emergency.publish(msg)
        
        if self.is_emergency:
            self.btn_emergency.config(text="RESET (Resume)", bg="orange")
            self.lbl_status.config(text="EMERGENCY!", fg="red")
        else:
            self.btn_emergency.config(text="EMERGENCY STOP", bg="red")
            self.lbl_status.config(text="IDLE", fg="black")

    def update_status(self, msg):
        if not self.is_emergency:
            self.lbl_status.config(text=msg.data, fg="black")

def main():
    print("[DEBUG] Init ROS...")
    rclpy.init()

    # 🚨 Stack Smashing 방지: argv 강제 초기화
    sys.argv = [sys.argv[0]]

    print("[DEBUG] Init GUI...")
    root = tk.Tk()
    app = LimoControlUI(root)
    
    print("[DEBUG] Start Loop...")
    try:
        root.mainloop()
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            app.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()