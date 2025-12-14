import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String, Float32, Bool
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class SmartDispatcher(Node):
    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---------------------------
        # Nav2 Navigator
        # ---------------------------
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # ---------------------------
        # 좌표 DB (네가 준 값 그대로)
        # ---------------------------
        self.coordinates = {
            "진단검사의학과": {"x": 0.4807, "y": 0.2763, "w": 1.0},
            "영상의학과": {"x": 6.5785, "y": 2.6214, "w": 1.0},
            "내과": {"x": 7.4453, "y": 0.5102, "w": 1.0},
            "정형외과": {"x": 0.7539, "y": -2.6409, "w": 1.0},
            "신경과": {"x": 2.8364, "y": 1.1752, "w": 1.0},
        }

        # ---------------------------
        # 상태 변수
        # ---------------------------
        self.current_goal_name = None
        self.current_goal_pose = None
        self.is_emergency = False

        # ---------------------------
        # 속도 상태 (YAML에서 읽어옴)
        # ---------------------------
        self.current_speed = self.get_initial_speed()
        self.min_speed = 0.10
        self.max_speed = 0.40
        self.step = 0.05

        # ---------------------------
        # Subscribers (UI → Dispatcher)
        # ---------------------------
        self.create_subscription(String, '/dispatch_target', self.dispatch_cb, 10)
        self.create_subscription(Float32, '/nav_speed_delta', self.speed_delta_cb, 10)
        self.create_subscription(Bool, '/nav_emergency', self.emergency_cb, 10)

        # ---------------------------
        # Publishers (Dispatcher → UI)
        # ---------------------------
        self.status_pub = self.create_publisher(String, '/nav_status', 10)
        self.speed_pub = self.create_publisher(Float32, '/nav_current_speed', 10)
        self.target_pub = self.create_publisher(String, '/nav_current_target', 10)

        self.publish_status("IDLE")

    # ======================================================
    # 목표 수신 → Nav2 주행
    # ======================================================
    def dispatch_cb(self, msg):
        if self.is_emergency:
            self.publish_status("STOPPED (EMERGENCY)")
            return

        name = msg.data
        if name not in self.coordinates:
            self.get_logger().error(f"Unknown target: {name}")
            return

        info = self.coordinates[name]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = info['x']
        pose.pose.position.y = info['y']
        pose.pose.orientation.w = info['w']

        self.current_goal_name = name
        self.current_goal_pose = pose

        self.target_pub.publish(String(data=name))
        self.publish_status(f"MOVING: {name}")

        self.navigator.goToPose(pose)

    # ======================================================
    # 속도 증감 (러닝머신 UP / DOWN)
    # ======================================================
    def speed_delta_cb(self, msg):
        self.current_speed += msg.data
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))
        self.apply_speed(self.current_speed)
        self.speed_pub.publish(Float32(data=self.current_speed))

    # ======================================================
    # Emergency Stop / Resume
    # ======================================================
    def emergency_cb(self, msg):
        if msg.data:
            self.is_emergency = True
            self.navigator.cancelTask()
            self.publish_status("STOPPED (EMERGENCY)")
        else:
            if self.current_goal_pose is not None:
                self.is_emergency = False
                self.publish_status(f"MOVING: {self.current_goal_name}")
                self.navigator.goToPose(self.current_goal_pose)

    # ======================================================
    # Nav2 파라미터 처리
    # ======================================================
    def get_initial_speed(self):
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        client.wait_for_service()
        req = GetParameters.Request(names=['max_velocity'])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        vel = future.result().values[0].double_array_value
        return vel[0]

    def apply_speed(self, speed):
        self.set_remote_param('/controller_server', 'FollowPath.max_vel_x', speed)
        self.set_remote_param('/velocity_smoother', 'max_velocity', [speed, 0.0, 1.0])

    def set_remote_param(self, node, name, value):
        client = self.create_client(SetParameters, f'{node}/set_parameters')
        client.wait_for_service()

        param = Parameter()
        param.name = name
        if isinstance(value, list):
            param.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE_ARRAY,
                double_array_value=value
            )
        else:
            param.value = ParameterValue(
                type=ParameterValue.TYPE_DOUBLE,
                double_value=float(value)
            )

        req = SetParameters.Request(parameters=[param])
        client.call_async(req)

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = SmartDispatcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
