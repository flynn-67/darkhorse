import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

APP_FILENAME = "doctor_app.py"

class DoctorUITrigger(Node):
    def __init__(self):
        super().__init__('doctor_ui_trigger')

        self.subscription = self.create_subscription(
            String,
            '/hospital/arrival_status',
            self.listener_callback,
            10
        )

        self.current_department = None
        self.process = None  # ✅ streamlit 프로세스 추적

        self.get_logger().info("✅ [UI Trigger] 병원 도착 신호 대기 중...")

    def listener_callback(self, msg):
        arrived_location = (msg.data or "").strip()
        if not arrived_location:
            return

        # 같은 진료과면 무시 (중복 실행 방지)
        if arrived_location == self.current_department:
            return

        self.current_department = arrived_location
        self.get_logger().info(f"📍 진료과 도착: {arrived_location}")

        self.trigger_app(arrived_location)

    def trigger_app(self, department):
        try:
            current_dir = os.path.dirname(os.path.realpath(__file__))
            app_path = os.path.join(current_dir, APP_FILENAME)

            # 👉 진료과를 환경변수로 전달
            env = os.environ.copy()
            env["CURRENT_DEPARTMENT"] = department

            # ✅ 이미 떠있는 streamlit이 있으면 종료 후 다시 띄우고 싶다면 (선택)
            # if self.process is not None and self.process.poll() is None:
            #     self.get_logger().info("🧹 이전 UI 프로세스 종료")
            #     self.process.terminate()

            self.get_logger().info(f"🚀 UI 실행 ({department}) : {app_path}")

            self.process = subprocess.Popen(
                ["bash", "-lc", "source ~/wego_ws/install/setup.bash && streamlit run " + app_path],
                env=env
            )

        except Exception as e:
            self.get_logger().error(f"❌ UI 실행 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoctorUITrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
