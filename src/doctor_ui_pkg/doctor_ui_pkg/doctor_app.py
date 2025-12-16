import os
import time
import streamlit as st

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

TOPIC_DOCTOR_DONE = "/hospital/doctor_input"

class DoctorUiRos(Node):
    def __init__(self):
        super().__init__("doctor_ui_streamlit_node")
        self.pub_done = self.create_publisher(Bool, TOPIC_DOCTOR_DONE, 10)

    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.pub_done.publish(msg)

def get_ros_node():
    """
    Streamlit은 rerun이 많아서 rclpy.init을 매번 하면 터질 수 있음.
    session_state에 ROS 노드를 1개만 유지.
    """
    if "ros_inited" not in st.session_state:
        rclpy.init(args=None)
        st.session_state.ros_inited = True

    if "ros_node" not in st.session_state:
        st.session_state.ros_node = DoctorUiRos()

    return st.session_state.ros_node

def main():
    st.set_page_config(page_title="Doctor UI", layout="wide")

    department = os.environ.get("CURRENT_DEPARTMENT", "진료과")
    st.title(f"👨‍⚕️ {department} 의료진 UI")

    # (여기에 너가 기존에 만들어둔 구글시트 조회 / 문진표 표시 / 진단서 작성 UI 그대로 두면 됨)

    st.divider()

    col1, col2 = st.columns([1, 1])

    with col1:
        st.subheader("진료 완료")
        if st.button("✅ 다음 진료로 이동", use_container_width=True):
            node = get_ros_node()
            node.publish_done()

            # publish가 바로 안 먹는 경우 대비 짧게 spin
            rclpy.spin_once(node, timeout_sec=0.1)

            st.success("로봇에게 '진료 완료' 신호를 보냈어. (doctor_input=True)")
            st.caption("BT의 WaitDoctorDone이 SUCCESS로 넘어가면서 다음 행동을 진행할 거야.")

    with col2:
        st.subheader("디버그")
        st.code(f"CURRENT_DEPARTMENT={department}\nPUBLISH={TOPIC_DOCTOR_DONE}", language="text")

if __name__ == "__main__":
    main()
