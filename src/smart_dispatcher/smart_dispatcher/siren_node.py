import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import pygame

class SirenNode(Node):
    def __init__(self):
        super().__init__('siren_node')

        # 1. 오디오 믹서 초기화 (가장 먼저)
        try:
            pygame.mixer.pre_init(44100, -16, 2, 2048)
            pygame.mixer.init()
            self.get_logger().info("✅ Audio Mixer Initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Mixer Init Error: {e}")

        self.siren_sound = None
        self.auto_off_timer = None # ROS Timer 사용

        # 2. 파일 로드
        try:
            package_share_directory = get_package_share_directory('smart_dispatcher')
            sound_path = os.path.join(package_share_directory, 'resource', 'siren.wav')
            
            if os.path.exists(sound_path):
                self.siren_sound = pygame.mixer.Sound(sound_path)
                self.get_logger().info(f"🔊 Sound Loaded: {sound_path}")
            else:
                self.get_logger().error(f"❌ File Missing: {sound_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Audio Load Error: {e}")

        # 3. 토픽 구독
        self.create_subscription(Bool, '/cmd_siren', self.cb_siren, 10)
        self.get_logger().info("📣 Siren Node Ready (ROS Timer + Nuclear Stop)")

    def cb_siren(self, msg: Bool):
        if not self.siren_sound: return

        if msg.data: # True: 켜기
            self.start_siren(10.0)
        else: # False: 끄기
            self.stop_siren()

    def start_siren(self, duration):
        # 1. 기존 소리 및 타이머 완전 제거
        self.stop_siren()

        # 2. 재생 시작
        self.get_logger().warn(f"🚨 SIREN ON ({duration}s)")
        try:
            self.siren_sound.play(loops=-1) # 무한 루프 재생
            
            # 3. ROS 2 타이머 생성 (10초 뒤 stop_siren 호출)
            # threading.Timer 대신 이걸 써야 안전합니다.
            self.auto_off_timer = self.create_timer(duration, self.stop_siren)
        except Exception as e:
            self.get_logger().error(f"Play Error: {e}")

    def stop_siren(self):
        # 1. 타이머가 있다면 즉시 삭제 (중복 실행 방지)
        if self.auto_off_timer:
            self.auto_off_timer.cancel()
            self.auto_off_timer.destroy()
            self.auto_off_timer = None

        # 2. [핵심] 믹서 전체 강제 정지 (Nuclear Option)
        # 특정 채널만 끄려다 실패하지 말고, 전체를 침묵시킵니다.
        if pygame.mixer.get_init():
            pygame.mixer.stop()
            self.get_logger().info("🔕 SIREN STOPPED (Nuclear)")

def main(args=None):
    rclpy.init(args=args)
    node = SirenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정리
        if node.auto_off_timer:
            node.auto_off_timer.cancel()
        pygame.mixer.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
