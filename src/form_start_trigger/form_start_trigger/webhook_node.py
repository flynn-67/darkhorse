import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from flask import Flask, request
import threading
import json
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import gspread
from oauth2client.service_account import ServiceAccountCredentials

# ==========================================
# [설정] 이메일 및 구글 시트 설정
# ==========================================
SENDER_EMAIL = "tlathwls0518@gmail.com"
SENDER_PASSWORD = "nhlp vrwa nkja pjom"
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
JSON_FILE_NAME = "service_account.json"  # 실행 위치에 이 파일이 있어야 함
# ==========================================

app = Flask(__name__)

class FormWebhookNode(Node):
    def __init__(self):
        super().__init__('form_webhook_node')

        # 1. Dispatcher(BT)로 데이터 전송
        self.pub_patient_data = self.create_publisher(
            String,
            '/hospital/mission_data',
            10
        )

        # 2. 미션 완료 신호 수신
        self.create_subscription(
            Bool,
            '/hospital/mission_completed',
            self.cb_mission_complete,
            10
        )

        # 현재 환자 정보 저장용
        self.current_patient = {}

        # Flask 라우트 설정
        @app.route('/form', methods=['POST'])
        def form_webhook():
            try:
                data = request.get_json(force=True)
                self.get_logger().info(f'📥 Webhook 데이터 수신: {data}')

                if not data:
                    return 'Bad Request', 400

                # 환자 정보 기억 (이메일 보낼 때 ID와 주소 필요)
                self.current_patient = data

                # ROS로 전송
                msg = String()
                msg.data = json.dumps(data, ensure_ascii=False)
                self.pub_patient_data.publish(msg)

                return 'OK', 200

            except Exception as e:
                self.get_logger().error(f'Error: {e}')
                return 'Error', 500

    def cb_mission_complete(self, msg):
        """BT로부터 완료 신호(True)가 오면 호출됨"""
        if msg.data is True:
            self.get_logger().info("✅ 미션 완료 신호 수신! 구글 시트 조회 및 이메일 작성 시작...")

            # 1. 기억해둔 환자 정보 확인
            email = self.current_patient.get("email")
            name = self.current_patient.get("name", "환자")
            p_id = self.current_patient.get("patient_id")  # "35" 같은 ID

            if not email:
                self.get_logger().warn("⚠️ 환자 이메일 정보가 없습니다.")
                return

            if not p_id:
                self.get_logger().warn("⚠️ 환자 ID가 없어 진료 기록을 조회할 수 없습니다.")
                # ID가 없어도 일단 기본 메일이라도 보냄
                self.send_email_to_patient(email, name, "진료 기록을 찾을 수 없습니다.")
                return

            # 2. 구글 시트에서 진료 기록 가져오기
            medical_history_text = self.fetch_medical_records(p_id)

            # 3. 이메일 발송
            self.send_email_to_patient(email, name, medical_history_text)

            # 초기화
            self.current_patient = {}

    def fetch_medical_records(self, patient_id):
        """구글 시트 '시트2'에서 해당 환자의 모든 기록을 가져와 텍스트로 변환"""
        try:
            scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
            creds = ServiceAccountCredentials.from_json_keyfile_name(JSON_FILE_NAME, scope)
            client = gspread.authorize(creds)

            # 시트 열기
            sheet = client.open("medical_records").worksheet("시트2")
            all_records = sheet.get_all_records()  # 딕셔너리 리스트 형태

            # 현재 환자 ID와 일치하는 기록만 필터링
            patient_records = [
                row for row in all_records
                if str(row.get('patient_id')) == str(patient_id) or str(row.get('ID')) == str(patient_id)
            ]

            if not patient_records:
                return "금일 진료 기록이 없습니다."

            # 텍스트로 예쁘게 포맷팅
            history_text = "\n[금일 진료 상세 내역]\n" + "=" * 30 + "\n"

            for idx, rec in enumerate(patient_records, 1):
                dept = rec.get('진료과', rec.get('Department', '-'))
                diag = rec.get('진단', rec.get('Diagnosis', '-'))
                pres = rec.get('처방', rec.get('Prescription', '-'))
                doc = rec.get('의사', rec.get('Doctor', '-'))

                history_text += f"{idx}. {dept}\n"
                history_text += f"    - 담당의: {doc}\n"
                history_text += f"    - 진단명: {diag}\n"
                history_text += f"    - 처방전: {pres}\n"
                history_text += "-" * 30 + "\n"

            return history_text

        except Exception as e:
            self.get_logger().error(f"구글 시트 읽기 실패: {e}")
            return f"진료 기록 시스템 접속 오류: {e}"

    def send_email_to_patient(self, target_email, target_name, history_body):
        try:
            subject = f"[Smart Hospital] {target_name}님, 진료 안내 및 처방 내역입니다."

            body = f"""
            안녕하세요, {target_name}님.

            스마트 병원 로봇입니다.
            요청하신 모든 진료 안내가 완료되었습니다.

            {history_body}

            오늘도 건강한 하루 보내시길 바랍니다.
            감사합니다.

            - Smart Hospital Robot 드림 -
            """

            msg = MIMEMultipart()
            msg['From'] = SENDER_EMAIL
            msg['To'] = target_email
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            server = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
            server.starttls()
            server.login(SENDER_EMAIL, SENDER_PASSWORD)
            server.sendmail(SENDER_EMAIL, target_email, msg.as_string())
            server.quit()

            self.get_logger().info(f'📧 이메일 전송 성공 -> {target_email}')

        except Exception as e:
            self.get_logger().error(f'❌ 이메일 전송 실패: {e}')

def main():
    rclpy.init()
    node = FormWebhookNode()

    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
        daemon=True
    ).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
