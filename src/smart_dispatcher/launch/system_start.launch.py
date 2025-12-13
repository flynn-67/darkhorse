import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # =========================================================
    # [사진 기반 경로 설정]
    # 1. 런치 파일은 'wego' 패키지에 있음
    nav_launch_pkg = 'wego'
    
    # 2. 맵 폴더는 'wego_2d_nav' 패키지에 있음
    map_pkg = 'wego_2d_nav'
    
    # 3. 맵 파일 이름 (maps 폴더 안에 있는 파일명으로 꼭 수정하세요!)
    map_file_name = 'map_1764225427.yaml' 
    # =========================================================

    # 맵 파일 절대 경로 생성 (wego_2d_nav 패키지에서 찾음)
    map_dir = os.path.join(
        get_package_share_directory(map_pkg),
        'maps',
        map_file_name
    )

    # 1. 네비게이션 실행 (wego 패키지에서 런치 파일 실행 + 맵 경로 전달)
    wego_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav_launch_pkg), 'launch', 'navigation_diff_launch.py')
        ),
        launch_arguments={'map': map_dir}.items()
    )

    # 2. 로봇 두뇌 (Smart Dispatcher) - 5초 뒤 실행
    dispatcher_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='smart_dispatcher',
                executable='dispatcher',
                name='smart_dispatcher',
                output='screen'
            )
        ]
    )

    # 3. 팀원 UI (Patient UI) - 8초 뒤 실행
    ui_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='smart_hospital_system',
                executable='patient_ui', 
                name='patient_ui_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        wego_nav_launch,
        dispatcher_node,
        ui_node
    ])