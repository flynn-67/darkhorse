import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from limo_interfaces.action import Speak
import pyttsx3


# Text to Speech Node / 텍스트 음성 변환 노드
class TextToSpeechNode(Node):

    #생성자 / Constructor
    def __init__(self):
        
        super().__init__('text_to_speech_node')
        self.get_logger().info('TTS Node has been started.')
        
        #엑션 서버로 변경
        self._action_server = ActionServer(
            self,
              Speak,
            'speak_text',
            self.execute_callback)

        # Initialize TTS Engine / TTS 엔진 초기화
        # 유지
        self.engine = pyttsx3.init()
        # Test Speaker / 스피커 테스트
        # 유지
        self.speak("System is ready.")

        self.get_logger().info('System is ready for TTS requests.')

        # test speak method
    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

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