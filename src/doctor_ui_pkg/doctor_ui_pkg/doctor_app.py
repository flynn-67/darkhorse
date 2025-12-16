# doctor_app.py
import streamlit as st
import gspread
from oauth2client.service_account import ServiceAccountCredentials
import pandas as pd
from datetime import datetime
import rclpy
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

        next_pub = node.create_publisher(Bool, '/hospital/next_waypoint', 10)
        return_pub = node.create_publisher(Bool, '/hospital/return_home', 10)
        doctor_done_pub = node.create_publisher(Bool, '/hospital/doctor_input', 10)

        # ✅ dispatcher/BT가 알려주는 "다음 waypoint 존재 여부"
        st.session_state['has_next_waypoint'] = True  # 기본값(못 받았을 때는 일단 다음으로 가게)
        st.session_state['last_has_next_update_ts'] = 0.0

        def has_next_cb(msg: Bool):
            st.session_state['has_next_waypoint'] = bool(msg.data)
            st.session_state['last_has_next_update_ts'] = time.time()

        node.create_subscription(Bool, '/hospital/has_next_waypoint', has_next_cb, 10)

        st.session_state['ros_node'] = node
        st.session_state['next_pub'] = next_pub
        st.session_state['return_pub'] = return_pub
        st.session_state['doctor_done_pub'] = doctor_done_pub

    return (
        st.session_state['ros_node'],
        st.session_state['next_pub'],
        st.session_state['return_pub'],
        st.session_state['doctor_done_pub']
    )

def pump_ros_callbacks(node):
    # Streamlit은 이벤트 루프가 없어서 콜백을 "가끔씩" 처리해줘야 함
    try:
        rclpy.spin_once(node, timeout_sec=0.01)
    except Exception:
        pass

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
    worksheet = sheet_file.worksheet("시트2")
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    worksheet.append_row([
        p_id,        # patient_id
        dept,        # 진료과
        diag,        # 진단
        "",          # 소견
        pres,        # 처방
        doc_name,    # 의사
        now_str,     # 작성 시간
        is_final     # 종료 여부(참고용)
    ])

def update_patient_status(sheet_file, p_id, status_msg):
    worksheet = sheet_file.worksheet("환자의 통합 데이터")
    try:
        cell = worksheet.find(str(p_id))
        if cell:
            status_col = 7  # G열
            worksheet.update_cell(cell.row, status_col, status_msg)
    except Exception as e:
        print(f"상태 업데이트 실패: {e}")

# ==========================================
# 2. Streamlit UI 구성
# ==========================================
st.set_page_config(page_title="🏥 병원 진료 시스템", layout="wide")
st.title("👨‍⚕️ 의사 전용 대시보드 (Doctor UI)")

node, next_pub, return_pub, doctor_done_pub = init_ros_node()
pump_ros_callbacks(node)  # ✅ 콜백 처리

# 화면에 현재 has_next 상태 표시(디버깅용)
has_next = st.session_state.get('has_next_waypoint', True)
last_ts = st.session_state.get('last_has_next_update_ts', 0.0)
st.caption(f"🛰 has_next_waypoint = {has_next}  (last update: {last_ts:.1f})")

try:
    sheet_file = connect_google_sheet()

    patient_sheet = sheet_file.worksheet("환자의 통합 데이터")
    data = patient_sheet.get_all_records()
    df = pd.DataFrame(data)

    # '완료'된 환자는 목록에서 제외
    if '진료상태' in df.columns:
        df = df[df['진료상태'] != '완료']

    st.sidebar.header("환자 대기 목록")

    if not df.empty and 'patient_id' in df.columns:
        patient_list = df['patient_id'].tolist()
        selected_patient_id = st.sidebar.selectbox("진료할 환자를 선택하세요", patient_list)

        patient_info = df[df['patient_id'] == selected_patient_id].iloc[0]
        patient_name = patient_info.get('이름', '이름없음')

        # -------------------------------
        # 상단: 환자 정보
        # -------------------------------
        col1, col2 = st.columns(2)
        with col1:
            st.info("### 📋 환자 정보")
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

        st.markdown("### 👇 진료 처리")

        # -------------------------------
        # 하단: 버튼 1개만 사용
        # -------------------------------
        if st.button("➡️ 다음 진료과로 이동", use_container_width=True):
            if not diagnosis:
                st.warning("진단 소견을 입력해주세요.")
            else:
                # 최신 has_next 한번 더 반영(버튼 누른 시점 콜백 처리)
                pump_ros_callbacks(node)
                has_next_now = st.session_state.get('has_next_waypoint', True)

                # 1) 기록 저장
                # has_next가 False면 사실상 "마지막 진료"라서 is_final=True로 저장
                save_to_sheet(
                    sheet_file,
                    selected_patient_id,
                    target_dept,
                    diagnosis,
                    prescription,
                    doctor_name,
                    is_final=(not has_next_now)
                )

                # 2) 환자 상태 업데이트 (G열)
                update_patient_status(sheet_file, selected_patient_id, "완료")

                # 3) ROS 메시지
                msg = Bool()
                msg.data = True

                # ✅ BT가 기다리는 신호(진료 완료)
                doctor_done_pub.publish(msg)

                # ✅ 다음 waypoint 있으면 next, 없으면 return_home
                if has_next_now:
                    next_pub.publish(msg)
                    st.success("🤖 로봇이 **다음 진료과**로 이동합니다.")
                else:
                    return_pub.publish(msg)
                    st.success("🤖 다음 진료과가 없어 **안내데스크(초기 위치)** 로 복귀합니다.")

                time.sleep(1.2)
                st.rerun()

    else:
        st.warning("대기 중인 환자가 없거나 데이터를 불러올 수 없습니다.")

except Exception as e:
    st.error(f"시스템 오류 발생: {e}")
