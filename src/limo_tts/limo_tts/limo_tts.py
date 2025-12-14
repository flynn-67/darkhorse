import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from limo_interfaces.action import Speak
import os  # pyttsx3 대신 os 사용

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.get_logger().info('TTS Node has been started.')
        
        self._action_server = ActionServer(
            self,
            Speak,
            'speak_text',
            self.execute_callback)

        # 초기 안내 음성
        self.speak("System is ready.")

    def speak(self, text):
        # [수정] pyttsx3 대신 리눅스 기본 명령어 espeak 사용 (훨씬 안정적)
        # 한국어를 쓰려면 espeak 뒤에 '-v ko' 옵션을 붙여야 할 수도 있습니다.
        # 영어 예시: os.system(f'espeak "{text}"')
        
        # 백그라운드 실행을 위해 & 를 붙이거나, 단순 실행
        # (한국어 TTS가 설치되어 있다면 'espeak -v ko' 사용)
        safe_text = text.replace('"', '\\"') # 따옴표 오류 방지
        os.system(f'espeak "{safe_text}"') 

    def execute_callback(self, goal_handle):
        text = goal_handle.request.text
        self.get_logger().info(f'Received TTS request: {text}')
        
        self.speak(text)
        
        goal_handle.succeed()
        result = Speak.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()