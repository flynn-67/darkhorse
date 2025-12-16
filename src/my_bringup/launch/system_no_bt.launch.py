from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # smart_dispatcher의 launch 파일 이름은 환경마다 다를 수 있어서 인자로 받게 함
    dispatcher_launch_file_arg = DeclareLaunchArgument(
        'dispatcher_launch_file',
        default_value='system_start.launch.py',  # 네가 수동으로 치던 이름
        description='Launch file name in smart_dispatcher/launch'
    )
    dispatcher_launch_file = LaunchConfiguration('dispatcher_launch_file')

    # 1) ros2 launch smart_dispatcher system_start_launch.py
    smart_dispatcher_share = get_package_share_directory('smart_dispatcher')
    dispatcher_launch_path = PathJoinSubstitution(
        [smart_dispatcher_share, 'launch', dispatcher_launch_file]
    )
    include_dispatcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dispatcher_launch_path)
    )

    # 2) ros2 launch my_bringup qr_launch.py
    my_bringup_share = get_package_share_directory('my_bringup')
    qr_launch_path = PathJoinSubstitution([my_bringup_share, 'launch', 'qr_launch.py'])
    include_qr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qr_launch_path)
    )

    # 3) ros2 run limo_tts limo_tts
    tts_node = Node(
        package='limo_tts',
        executable='limo_tts',
        name='limo_tts',
        output='screen'
    )

    # 4) ros2 run smart_dispatcher siren_node
    siren_node = Node(
        package='smart_dispatcher',
        executable='siren_node',
        name='siren_node',
        output='screen'
    )

    # 5) ros2 run smart_dispatcher ui_node
    ui_node = Node(
        package='smart_dispatcher',
        executable='ui_node',
        name='ui_node',
        output='screen'
    )

    # 6) ros2 run form_start_trigger webhook_node
    webhook_node = Node(
        package='form_start_trigger',
        executable='webhook_node',
        name='webhook_node',
        output='screen'
    )

    # 7) ros2 run doctor_ui_pkg doctor_ui_trigger
    doctor_trigger_node = Node(
        package='doctor_ui_pkg',
        executable='doctor_ui_trigger',
        name='doctor_ui_trigger',
        output='screen'
    )

    return LaunchDescription([
        dispatcher_launch_file_arg,
        include_dispatcher,
        include_qr,
        tts_node,
        siren_node,
        ui_node,
        webhook_node,
        doctor_trigger_node,
    ])
