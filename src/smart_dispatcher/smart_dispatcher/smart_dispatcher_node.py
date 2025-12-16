#!/usr/bin/env python3
import json
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


# ✅ 좌표는 네 맵에 맞는 걸로 유지/추가하면 됨
DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": -2.0478696823120117, "y": 1.3148077726364136, "w": 1.0},
    "정형외과":      {"x": 4.325248718261719,  "y": -1.067739486694336, "w": 1.0},
    "안내데스크":    {"x": 0.08828259259462357,"y": 0.08828259259462357,"w": 1.0},
}
INFO_DESK_NAME = "안내데스크"


class SmartDispatcher(Node):
    """
    /hospital/patient_data(JSON) -> departments 중에서 안내데스크 제외하고 후보 생성
    출발할 때마다 랜덤 대기인원 생성 -> 최소 대기인원 과로 이동
    waypoint 도착 후 /hospital/next_waypoint(True) 오면 다음 출발

    + 도착 성공 시 /hospital/arrival_status(String)에 현재 과 이름 publish
    + ✅ 다음 waypoint 존재 여부를 /hospital/has_next_waypoint(Bool) publish
    + ✅ UI가 '복귀'를 요청하면 /hospital/return_home(Bool) 받고 안내데스크 복귀
    """

    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---- 상태 ----
        self.remaining_depts = []
        self.waiting_counts = {}
        self.wait_min = 0
        self.wait_max = 20

        self.current_goal_name = None
        self.current_goal_pose = None
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False

        # ✅ 복귀 상태
        self.is_returning_home = False
        self.return_requested = False

        # ---- home 저장 ----
        self.home_pose = None
        self.home_saved = False
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)

        # ---- Nav2 ----
        self.navigator = BasicNavigator()
        try:
            self.navigator.waitUntilNav2Active()
        except Exception as e:
            self.get_logger().warn(f"waitUntilNav2Active() 예외: {e}")

        # ---- 속도 ----
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # ---- Pub ----
        self.pub_arrival_status = self.create_publisher(String, '/hospital/arrival_status', 10)
        self.pub_has_next = self.create_publisher(Bool, '/hospital/has_next_waypoint', 10)

        # ---- Sub ----
        self.create_subscription(String,  '/hospital/patient_data',  self.cb_patient_data, 10)
        self.create_subscription(Bool,    '/hospital/next_waypoint', self.cb_next_waypoint, 10)
        self.create_subscription(Bool,    '/hospital/return_home',   self.cb_return_home, 10)  # ✅ 추가
        self.create_subscription(Float32, '/nav_speed_delta',        self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',              self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',     self.cb_emergency_home, 10)

        self.get_logger().info("IDLE: QR 대기 중 (dispatcher ready)")
        self.create_timer(0.1, self.loop)

    # ---------------- 콜백 ----------------
    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        if self.home_saved:
            return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose
        self.home_pose = pose
        self.home_saved = True
        self.get_logger().info("[dispatcher] Home pose saved")

    def cb_patient_data(self, msg: String):
        if self.is_emergency or self.is_returning_home:
            self.get_logger().info("BUSY: emergency/returning 중 (patient_data 무시)")
            return

        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
        except Exception as e:
            self.get_logger().error(f"patient_data JSON parse fail: {e}")
            return

        self.remaining_depts = [
            d for d in depts
            if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)
        ]

        if not self.remaining_depts:
            self.get_logger().info("IDLE: 이동할 waypoint 없음 (안내데스크는 후보 제외)")
            self._publish_has_next(False)
            return

        self.get_logger().info("READY: 첫 목적지 출발(최소 대기인원)")
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False
        self.is_returning_home = False
        self.return_requested = False

        self._publish_has_next(True)
        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        if not msg.data:
            return
        if self.is_emergency or self.is_returning_home:
            return
        if self.waiting_next:
            self.waiting_next = False
            self.get_logger().info("MOVING: 다음 목적지 출발(최소 대기인원)")
            self._start_next_goal()

    def cb_return_home(self, msg: Bool):
        """✅ UI가 마지막이면 True를 보내는 토픽"""
        if not msg.data:
            return
        if self.is_emergency:
            return

        self.get_logger().info("RETURN_HOME: UI 요청 수신 -> 안내데스크 복귀 시작")
        self.return_requested = True

        # 현재 이동중이면 취소하고 복귀
        try:
