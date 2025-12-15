import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== 기본값(네 환경에 맞게 경로만 수정 가능) =====
    default_map = os.environ.get(
        "HOSPITAL_MAP_YAML",
        "/home/flynn/darkhorse/src/config/map_67.yaml"
    )
    default_waypoints = os.environ.get(
        "HOSPITAL_WAYPOINTS_FILE",
        "/home/flynn/darkhorse/src/config/hospital_waypoints.yaml"
    )

    # wego 네비게이션 런치 패키지(이건 너 환경에 있는 걸로 유지)
    nav_launch_pkg = "wego"
    nav_launch_file = "navigation_diff_launch.py"

    # launch args
    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    waypoints_arg = DeclareLaunchArgument("waypoints", default_value=default_waypoints)

    # Nav2 bringup (wego 패키지 런치 include)
    wego_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav_launch_pkg), "launch", nav_launch_file)
        ),
        launch_arguments={"map": LaunchConfiguration("map")}.items()
    )

    # Dispatcher 노드(너 setup.py entry_points: dispatcher)
    dispatcher_node = Node(
        package="smart_dispatcher",
        executable="dispatcher",
        name="smart_dispatcher",
        output="screen",
        parameters=[{
            "waypoint_file": LaunchConfiguration("waypoints"),
        }],
    )

    # Nav2 올라온 뒤 dispatcher 실행되게 약간 지연(선택이지만 추천)
    delayed_dispatcher = TimerAction(period=2.0, actions=[dispatcher_node])

    return LaunchDescription([
        map_arg,
        waypoints_arg,
        wego_nav_launch,
        delayed_dispatcher,
    ])
