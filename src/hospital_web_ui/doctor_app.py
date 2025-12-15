import streamlit as st
import gspread
from oauth2client.service_account import ServiceAccountCredentials
import pandas as pd
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

# ==========================================
# 0. ROS 2 노드 설정 (Streamlit 전용)
# ==========================================
def init_ros_node():
    if not rclpy.ok():
        rclpy.init()

    if 'ros_node' not in st.session_state:
        node = rclpy.create_node('streamlit_doctor_node')

        # 의미 단위로 토픽 분리
        next_pub = node.create_publisher(Bool, '/hospital/next_waypoint', 10)
        return_pub = node.create_publisher(Bool, '/hospital/return_home', 10)

        st.session_state['ros_node'] = node
        st.session_state['next_pub'] = next_pub
        st.session_state['return_pub'] = return_pub

    return (
        st.session_state['ros_node'],
        st.session_state['next_pub'],
        st.session_state['return_pub']
    )

# ==========================================
# 1. 구글 시트 관련 함수
# ==========================================
def connect_google_sheet():
    scope = [
        "https://spreadsheets.google.com/feeds",
        "https://www.googleapis.com/auth/drive"
    ]
    creds = ServiceAccountCredentials.from_json_keyfile_name(
        "service_account.json", scope
    )
    client = gspread.authorize(creds)
    sheet = client.open("medical_records")
    return sheet

def save_to_sheet(sheet_file, p_id, dept, diag, pres, doc_name, is_final):
    """
    진료 기록 저장
    is_final = True  -> 모든 진료 종료 (이메일 발송 대상)
    is_final = False -> 다음 진료과 이동
    """
    worksheet = sheet_file.worksheet("시트2")
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    worksheet.append_row([
        p_id,        # patient_id
        dept,        # 진료과
        diag,        # 진단
        "",          # 소견 (비워둠)
        pres,        # 처방
        doc_name,    # 의사
        now_str,     # 작성 시간
        is_final     # 이메일/종료 여부
    ])

# ==========================================
# 2. Streamlit UI 구성
# ==========================================
st.set_page_config(page_title="🏥 병원 진료 시스템", layout="wide")
st.title("👨‍⚕️ 의사 전용 대시보드 (Doctor UI)")

# ROS 초기화
node, next_pub, return_pub = init_ros_node()

try:
    # 구글 시트 연결
    sheet_file = connect_google_sheet()

    # 환자 목록 불러오기
    patient_sheet = sheet_file.worksheet("환자의 통합 데이터")
    data = patient_sheet.get_all_records()
    df = pd.DataFrame(data)

    st.sidebar.header("환자 대기 목록")

    if not df.empty and 'patient_id' in df.columns:
        patient_list = df['patient_id'].tolist()
        selected_patient_id = st.sidebar.selectbox(
            "진료할 환자를 선택하세요", patient_list
        )

        patient_info = df[df['patient_id'] == selected_patient_id].iloc[0]
        patient_name = patient_info.get('이름', '이름없음')

        # -------------------------------
        # 상단: 환자 정보
        # -------------------------------
        col1, col2 = st.columns(2)
        with col1:
            st.info(f"### 📋 환자 정보")
            st.write(f"**이름:** {patient_name}")
            st.write(f"**ID:** {selected_patient_id}")
            st.write(f"**성별:** {patient_info.get('성별', '-')}")
            st.write(f"**나이:** {patient_info.get('나이', '-')}")
        with col2:
            st.error("### 🚨 주요 증상")
            st.write(patient_info.get('증상', '내용 없음'))

        st.markdown("---")

        # -------------------------------
        # 중앙: 진료 입력
        # -------------------------------
        st.subheader("📝 진료 기록 작성")

        c1, c2 = st.columns(2)
        with c1:
            doctor_name = st.text_input("담당 의사", value="김닥터")
            target_dept = st.text_input("현재 진료과", value="내과")
        with c2:
            diagnosis = st.text_area("진단 소견", height=120)
            prescription = st.text_area("처방 내용", height=120)

        st.markdown("### 👇 진료 처리 선택")

        # -------------------------------
        # 하단: 액션 버튼
        # -------------------------------
        b1, b2 = st.columns(2)

        # ▶ 다음 진료과 이동
        with b1:
            if st.button("➡️ 다음 진료과로 이동", use_container_width=True):
                if not diagnosis:
                    st.warning("진단 소견을 입력해주세요.")
                else:
                    save_to_sheet(
                        sheet_file,
                        selected_patient_id,
                        target_dept,
                        diagnosis,
                        prescription,
                        doctor_name,
                        is_final=False
                    )

                    msg = Bool()
                    msg.data = True
                    next_pub.publish(msg)

                    st.success("🤖 로봇이 **다음 진료과**로 이동합니다.")
                    time.sleep(1.5)
                    st.rerun()

        # ✅ 모든 진료 종료 → 이메일 + 복귀
        with b2:
            if st.button(
                "✅ 모든 진료 종료 (이메일 & 복귀)",
                type="primary",
                use_container_width=True
            ):
                if not diagnosis:
                    st.warning("진단 소견을 입력해주세요.")
                else:
                    save_to_sheet(
                        sheet_file,
                        selected_patient_id,
                        target_dept,
                        diagnosis,
                        prescription,
                        doctor_name,
                        is_final=True
                    )

                    msg = Bool()
                    msg.data = True
                    return_pub.publish(msg)

                    st.success(
                        f"[{patient_name}]님 진료 종료 ✔️\n"
                        "📧 이메일 발송 및 🏠 초기 위치 복귀를 요청했습니다."
                    )
                    st.balloons()
                    time.sleep(2)
                    st.rerun()

    else:
        st.warning("대기 중인 환자가 없거나 데이터를 불러올 수 없습니다.")

except Exception as e:
    st.error(f"시스템 오류 발생: {e}")
