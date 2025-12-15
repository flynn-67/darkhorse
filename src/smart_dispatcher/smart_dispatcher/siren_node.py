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
        pygame.mixer.init()
        self.channel = None
        self.is_playing = False
        self.siren_sound = None

        # 2. 파일 경로 찾기 (설치된 share 폴더 기준)
        try:
            package_share_directory = get_package_share_directory('smart_dispatcher')
            sound_path = os.path.join(package_share_directory, 'resource', 'siren.wav')
            
            if os.path.exists(sound_path):
                self.siren_sound = pygame.mixer.Sound(sound_path)
                self.get_logger().info(f"🔊 Sound Loaded: {sound_path}")
            else:
                self.get_logger().error(f"❌ File Missing: {sound_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Audio Error: {e}")

        # 3. 명령 대기 (/cmd_siren)
        self.create_subscription(Bool, '/cmd_siren', self.cb_siren, 10)
        self.get_logger().info("📣 Siren Node Ready & Waiting...")

    def cb_siren(self, msg: Bool):
        if not self.siren_sound:
            return

        should_play = msg.data

        if should_play:
            if not self.is_playing:
                self.get_logger().warn("🚨 SIREN ON")
                self.channel = self.siren_sound.play(loops=-1) # 무한반복
                self.is_playing = True
        else:
            if self.is_playing:
                self.get_logger().info("🔕 SIREN OFF")
                if self.channel:
                    self.channel.stop()
                self.is_playing = False

def main(args=None):
    rclpy.init(args=args)
    node = SirenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()