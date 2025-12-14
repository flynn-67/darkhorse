import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import String, Float32, Bool
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


class SmartDispatcher(Node):
    def __init__(self):
        super().__init__('smart_dispatcher')

        # -------------------------
        # 좌표 DB (네가 준 값 그대로)
        # -------------------------
        self.coordinates = {
            "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
            "영상의학과":    {"x": 6.578537940979004,  "y": 2.621462106704712,  "w": 1.0},
            "내과":          {"x": 7.445363998413086,  "y": 0.5102964639663696, "w": 1.0},
            "정형외과":      {"x": 0.753912627696991,  "y": -2.640972375869751, "w": 1.0},
            "신경과":        {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
        }

        # -------------------------
        # 상태 변수
        # -------------------------
        self.current_goal_name = None
        self.current_goal_pose = None
        self.is_emergency = False

        # -------------------------
        # Nav2 Navigator
        # -------------------------
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        # -------------------------
        # 속도 상태 (Nav2 YAML에서 읽어서 시작)
        # -------------------------
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40
        self.step = 0.05

        # -------------------------
        # Sub: UI/BT → Dispatcher
        # -------------------------
        self.create_subscription(String,  '/dispatch_target',   self.cb_target,   10)
        self.create_subscription(Float32, '/nav_speed_delta',   self.cb_speed,    10)
        self.create_subscription(Bool,    '/nav_emergency',     self.cb_emergency,10)

        # -------------------------
        # Pub: Dispatcher → UI
        # -------------------------
        self.pub_status = self.create_publisher(String,  '/nav_status',         10)
        self.pub_target = self.create_publisher(String,  '/nav_current_target', 10)
        self.pub_speed  = self.create_publisher(Float32, '/nav_current_speed',  10)
        self.pub_goal   = self.create_publisher(PoseStamped, '/nav_goal_pose',  10)

        self._publish_status("IDLE")
        self.pub_speed.publish(Float32(data=float(self.current_speed)))

    # ========== 목표 수신 → goToPose ==========
    def cb_target(self, msg: String):
        if self.is_emergency:
            self._publish_status("STOPPED (EMERGENCY)")
            return

        name = msg.data.strip()
        if name not in self.coordinates:
            self.get_logger().error(f"[dispatcher] Unknown target: {name}")
            return

        info = self.coordinates[name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(info["x"])
        pose.pose.position.y = float(info["y"])
        pose.pose.orientation.w = float(info["w"])

        self.current_goal_name = name
        self.current_goal_pose = pose

        self.pub_target.publish(String(data=name))
        self.pub_goal.publish(pose)

        self._publish_status(f"MOVING: {name}")
        self.navigator.goToPose(pose)

    # ========== 속도 증감(UP/DOWN) ==========
    def cb_speed(self, msg: Float32):
        # delta 적용
        self.current_speed = float(self.current_speed) + float(msg.data)
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))

        # Nav2 파라미터 반영
        self._apply_speed(self.current_speed)
        self.pub_speed.publish(Float32(data=float(self.current_speed)))

    # ========== 비상정지/재개 ==========
    def cb_emergency(self, msg: Bool):
        if msg.data:  # True -> STOP
            self.is_emergency = True
            self.navigator.cancelTask()
            self._publish_status("STOPPED (EMERGENCY)")
        else:         # False -> RESUME
            if self.current_goal_pose is not None:
                self.is_emergency = False
                self._publish_status(f"MOVING: {self.current_goal_name}")
                self.navigator.goToPose(self.current_goal_pose)
            else:
                self.is_emergency = False
                self._publish_status("IDLE")

    # ========== 초기 속도 읽기 ==========
    def _get_initial_speed_from_velocity_smoother(self) -> float:
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        client.wait_for_service()
        req = GetParameters.Request()
        req.names = ['max_velocity']
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        try:
            arr = fut.result().values[0].double_array_value
            return float(arr[0]) if len(arr) > 0 else 0.25
        except Exception:
            return 0.25

    # ========== 속도 적용 (중요: velocity_smoother + controller 둘 다) ==========
    def _apply_speed(self, speed: float):
        # controller_server (DWB)
        self._set_remote_param('/controller_server', 'FollowPath.max_vel_x', speed)
        # velocity_smoother (최종 상한)
        self._set_remote_param('/velocity_smoother', 'max_velocity', [speed, 0.0, 1.0])

    def _set_remote_param(self, node_name: str, param_name: str, value):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        client.wait_for_service()

        p = Parameter()
        p.name = param_name

        if isinstance(value, list):
            p.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE_ARRAY,
                double_array_value=[float(x) for x in value]
            )
        else:
            p.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE,
                double_value=float(value)
            )

        req = SetParameters.Request()
        req.parameters = [p]
        client.call_async(req)

    def _publish_status(self, s: str):
        self.pub_status.publish(String(data=s))
        self.get_logger().info(s)


def main():
    rclpy.init()
    node = SmartDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

