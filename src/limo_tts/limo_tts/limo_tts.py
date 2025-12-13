import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from limo_interfaces.action import Speak
import pyttsx3
import threading
import queue
import time


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
        self.tts_queue = queue.Queue()

        # Test Speaker / 스피커 테스트
        # 유지
        self.speak("System is ready.")

        # 주석 처리 후 테스트
        # 별도 스레드에서 TTS 처리 시작
        self.thread = threading.Thread(target=self.process_tts_queue)
        self.thread.daemon = True # 노드 종료 시 스레드도 자동 종료 / Auto-kill thread when node exits
        self.thread.start()

        self.get_logger().info('TTS Node has been started. Waiting for /voice_cmd...')

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def listener_callback(self, msg):
        """
        메시지를 받으면 바로 재생하지 않고 큐에 쌓아둡니다. (비동기 처리)
        When a message is received, it is put into the queue instead of playing immediately. (Asynchronous)
        """
        self.get_logger().info(f'Received text: "{msg.data}"')
        self.tts_queue.put(msg.data)

    def process_tts_queue(self):
        """
        백그라운드에서 큐를 감시하며 텍스트를 읽어주는 함수
        Function that monitors the queue in the background and reads text
        """
        while rclpy.ok():
            try:
                # 큐에서 텍스트를 가져옴 (데이터가 없으면 1초 대기)
                # Get text from queue (wait 1 sec if empty)
                text = self.tts_queue.get(block=True, timeout=1.0)
                
                if text:
                    # 실제 음성 재생 (이 부분에서 시간이 걸려도 로봇은 멈추지 않음)
                    # Actual playback (Robot doesn't freeze even if this takes time)
                    self.engine.say(text)
                    self.engine.runAndWait()
                    self.tts_queue.task_done()
                    
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'TTS Error: {str(e)}')

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