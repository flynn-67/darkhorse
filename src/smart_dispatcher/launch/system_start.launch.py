import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # =========================================================
    # [사진 기반 경로 설정]
    nav_launch_pkg = 'wego'
    map_pkg = 'wego_2d_nav'
    map_file_name = 'map_67.yaml' 
    # =========================================================

    map_dir = os.path.join(
        get_package_share_directory(map_pkg),
        'maps',
        map_file_name
    )

    # 1. 네비게이션 실행 (Nav2)
    wego_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav_launch_pkg), 'launch', 'navigation_diff_launch.py')
        ),
        launch_arguments={'map': map_dir}.items()
    )




    return LaunchDescription([
        wego_nav_launch,
    ])