import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import pygame

class SirenNode(Node):
    def __init__(self):
        super().__init__('siren_node')

        # 1. 오디오 초기화
        try:
            pygame.mixer.init()
            self.get_logger().info("✅ Audio Mixer Initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Mixer Init Error: {e}")

        self.siren_sound = None
        self.auto_off_timer = None 

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
        self.get_logger().info("📣 Siren Node Ready (Nuclear Stop Mode)")

    def cb_siren(self, msg: Bool):
        if not self.siren_sound: return

        if msg.data: # True: 켜기
            self.start_siren(10.0)
        else: # False: 끄기
            self.stop_siren()

    def start_siren(self, duration):
        # 1. 일단 끄고 시작 (중복 방지)
        self.stop_siren()

        # 2. 재생 시작
        self.get_logger().warn(f"🚨 SIREN ON ({duration}s)")
        try:
            self.siren_sound.play(loops=-1) # 무한 반복 재생
        except Exception as e:
            self.get_logger().error(f"Play Error: {e}")
        
        # 3. ROS 타이머 생성 (10초 뒤 강제 종료)
        self.auto_off_timer = self.create_timer(duration, self.stop_siren)

    def stop_siren(self):
        # 1. 타이머 제거
        if self.auto_off_timer:
            self.auto_off_timer.cancel()
            self.auto_off_timer.destroy()
            self.auto_off_timer = None

        # 2. [핵심] 믹서 전체 정지 (모든 채널 강제 침묵)
        # channel 변수를 쓰지 않고, 믹서 자체를 멈춥니다. 좀비 소리까지 다 죽습니다.
        if pygame.mixer.get_init():
            pygame.mixer.stop() 
            self.get_logger().info("🔕 SIREN KILLED (Mixer Stop)")

def main(args=None):
    rclpy.init(args=args)
    node = SirenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.auto_off_timer:
            node.auto_off_timer.cancel()
        # 종료 시 확실하게 믹서 종료
        pygame.mixer.stop()
        pygame.mixer.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()