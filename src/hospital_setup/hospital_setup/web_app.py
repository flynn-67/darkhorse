import os
import streamlit as st

WAYPOINT_PATH = os.path.expanduser("~/.ros/hospital_waypoints.yaml")

# 너가 만든 공용 모듈 import (경로에 맞게)
from shared.hospital_waypoints import get_waypoints, save_waypoints

st.title("병원 웨이포인트 설정")

data = get_waypoints(WAYPOINT_PATH)
depts = data.get("departments", {})

# 표 형태로 편집
st.subheader("진료과 좌표 편집 (x, y, yaw)")
names = list(depts.keys())
if not names:
    st.warning("departments가 비어있음. YAML에 departments를 먼저 넣어줘.")
else:
    # 편집용 dict 복사
    edited = {}
    for name in names:
        col1, col2, col3, col4 = st.columns([2,2,2,2])
        with col1:
            st.write(f"**{name}**")
        with col2:
            x = st.number_input(f"{name} x", value=float(depts[name].get("x", 0.0)), key=f"{name}_x")
        with col3:
            y = st.number_input(f"{name} y", value=float(depts[name].get("y", 0.0)), key=f"{name}_y")
        with col4:
            yaw = st.number_input(f"{name} yaw(rad)", value=float(depts[name].get("yaw", 0.0)), key=f"{name}_yaw")
        edited[name] = {"x": float(x), "y": float(y), "yaw": float(yaw)}

    if st.button("✅ 저장"):
        save_waypoints(WAYPOINT_PATH, edited)
        st.success(f"저장 완료: {WAYPOINT_PATH}")
        st.rerun()
