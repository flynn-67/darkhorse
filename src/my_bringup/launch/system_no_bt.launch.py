'''from launch import LaunchDescription
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
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------------
    # [설정] 맵 파일 경로 (파일이 실제로 존재하는지 꼭 확인하세요!)
    # ---------------------------------------------------------
    map_file = '/home/wego/darkhorse/maps/map_1764225427.yaml'
    
    wego_share = get_package_share_directory('wego')
    my_bringup_share = get_package_share_directory('my_bringup')

    # =========================================================
    # 1. 로봇 하드웨어 실행 (Teleop)
    # ⚠️ 중요: 여기서 Rviz가 켜지면 안 되므로 viz='false' 전달
    # =========================================================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wego_share, 'launch', 'teleop_launch.py')
        ),
        launch_arguments={
            'viz': 'false'  # Teleop의 Rviz는 끕니다 (충돌 방지)
        }.items()
    )

    # =========================================================
    # 2. 네비게이션 실행 (AMCL / Map Server)
    # =========================================================
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wego_share, 'launch', 'navigation_diff_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false' # 실물 로봇이므로 false
        }.items()
    )

    # ... (나머지 노드들: QR, TTS, Siren, UI, Webhook, Doctor 등) ...
    # 기존 코드 그대로 유지
    qr_launch_path = PathJoinSubstitution([my_bringup_share, 'launch', 'qr_launch.py'])
    include_qr = IncludeLaunchDescription(PythonLaunchDescriptionSource(qr_launch_path))
    
    tts_node = Node(package='limo_tts', executable='limo_tts', name='limo_tts', output='screen')
    siren_node = Node(package='smart_dispatcher', executable='siren_node', name='siren_node', output='screen')
    
    ui_node = TimerAction(
        period=5.0,
        actions=[Node(package='smart_dispatcher', executable='ui_node', name='ui_node', output='screen')]
    )
    
    webhook_node = Node(package='form_start_trigger', executable='webhook_node', name='webhook_node', output='screen')
    doctor_trigger_node = Node(package='doctor_ui_pkg', executable='doctor_ui_trigger', name='doctor_ui_trigger', output='screen')

    return LaunchDescription([
        teleop_launch,
        nav_launch,
        include_qr,
        tts_node,
        siren_node,
        ui_node,
        webhook_node,
        doctor_trigger_node,
    ])