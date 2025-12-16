import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import pygame


class SirenNode(Node):
    def __init__(self):
        super().__init__('siren_node')

        # 1) 오디오 믹서 초기화 (가장 먼저)
        try:
            pygame.mixer.pre_init(44100, -16, 2, 2048)
            pygame.mixer.init()
            self.get_logger().info("✅ Audio Mixer Initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Mixer Init Error: {e}")

        self.siren_sound = None
        self.auto_off_timer = None  # ROS Timer 사용

        # ✅ 상태 플래그: 중복 True/False 방지
        self.is_on = False

        # 2) 파일 로드
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

        # 3) 토픽 구독
        self.create_subscription(Bool, '/cmd_siren', self.cb_siren, 10)
        self.get_logger().info("📣 Siren Node Ready (ROS Timer + Nuclear Stop)")

    def cb_siren(self, msg: Bool):
        if not self.siren_sound:
            return

        # ✅ 중복 True/False 방어: 이미 같은 상태면 무시
        if msg.data is True:
            if self.is_on:
                self.get_logger().info("🔁 Siren already ON -> ignore duplicate True")
                return
            self.is_on = True
            self.start_siren(10.0)

        else:  # False
            if not self.is_on:
                self.get_logger().info("🔁 Siren already OFF -> ignore duplicate False")
                return
            self.is_on = False
            self.stop_siren()

    def start_siren(self, duration: float):
        # ✅ 이미 켜져있으면 재시작 금지
        # (cb_siren에서 이미 막지만, 안전하게 2중 방어)
        if self.is_on and self.auto_off_timer is not None:
            self.get_logger().info("🔁 start_siren ignored (already running)")
            return

        # 1) 기존 소리 및 타이머 완전 제거
        self._cancel_timer_only()
        self._stop_audio_only()

        # 2) 재생 시작
        self.get_logger().warn(f"🚨 SIREN ON ({duration}s)")
        try:
            self.siren_sound.play(loops=-1)  # 무한 루프 재생

            # 3) ROS 2 타이머 생성 (duration초 뒤 stop_siren 호출)
            # create_timer는 주기 타이머지만, stop_siren에서 cancel/destroy하므로 1회성처럼 사용 가능
            self.auto_off_timer = self.create_timer(duration, self.stop_siren)

        except Exception as e:
            self.get_logger().error(f"Play Error: {e}")
            self.is_on = False

    def stop_siren(self):
        # 1) 타이머 즉시 삭제 (중복 실행 방지)
        self._cancel_timer_only()

        # 2) [핵심] 믹서 전체 강제 정지 (Nuclear Option)
        self._stop_audio_only()

        # 3) 상태 정리
        self.is_on = False

    # -------------------------
    # 내부 유틸
    # -------------------------
    def _cancel_timer_only(self):
        if self.auto_off_timer:
            try:
                self.auto_off_timer.cancel()
            except Exception:
                pass
            try:
                self.auto_off_timer.destroy()
            except Exception:
                pass
            self.auto_off_timer = None

    def _stop_audio_only(self):
        if pygame.mixer.get_init():
            try:
                pygame.mixer.stop()
            except Exception:
                pass
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
        try:
            node.stop_siren()
        except Exception:
            pass

        try:
            pygame.mixer.quit()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
