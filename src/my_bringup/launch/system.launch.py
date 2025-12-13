import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------------
    # [설정] 맵 파일 경로
    # ---------------------------------------------------------
    map_file = '/home/wego/darkhorse/maps/map_1764225427.yaml'

    # =========================================================
    # 0. [LIMO] 로봇 하드웨어/구동계 실행 (텔레옵) <--- 추가됨!
    # =========================================================
    # 선생님이 첫 번째 터미널에서 치시던 명령어입니다.
    wego_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wego'), 'launch', 'teleop_launch.py')
        )
    )

    # =========================================================
    # 1. [LIMO] 네비게이션 실행 (지도 + 길찾기)
    # =========================================================
    wego_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wego'), 'launch', 'navigation_diff_launch.py')
        ),
        launch_arguments={'map': map_file}.items()
    )

    # =========================================================
    # 2. [Brain] Smart Dispatcher 실행 (5초 딜레이)
    # =========================================================
    dispatcher_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="🚀 [System] 스마트 로봇 두뇌 가동..."),
            Node(
                package='smart_dispatcher',
                executable='dispatcher',
                name='smart_dispatcher',
                output='screen'
            )
        ]
    )

    # =========================================================
    # 3. [UI] Smart Hospital System 실행 (8초 딜레이)
    # =========================================================
    patient_ui_node = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="🖥️ [System] 환자용 키오스크 실행..."),
            Node(
                package='smart_hospital_system',
                executable='patient_ui',
                name='patient_ui',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        wego_bringup_launch, # 0. 하드웨어 ON
        wego_nav_launch,     # 1. 지도 ON
        dispatcher_node,     # 2. 두뇌 ON
        patient_ui_node      # 3. UI ON
    ])