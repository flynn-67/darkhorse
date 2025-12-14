import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Orbbec 카메라 런치 파일 경로 가져오기
    # (ros2 launch orbbec_camera astra_stereo_u3.launch.py)
    camera_launch_dir = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch')
    
    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_launch_dir, 'astra_stereo_u3.launch.py')
        )
    )

    # 2. Wego Teleop 런치 파일 경로 가져오기
    # (ros2 launch wego teleop_launch.py)
    # 만약 'wego' 패키지 안에 launch 폴더가 있다면 아래처럼 경로를 지정합니다.
    wego_launch_dir = os.path.join(
        get_package_share_directory('wego'), 'launch')

    teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wego_launch_dir, 'teleop_launch.py')
        )
    )

    # 3. QR 스캐너 노드 설정
    # (ros2 run qr_scanner qr_registration)
    qr_node = Node(
        package='qr_scanner',
        executable='qr_registration',
        name='qr_registration_node',
        output='screen'
    )

    # 4. RQT Image View 노드 설정
    # (ros2 run rqt_image_view rqt_image_view)
    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/camera/rgb/image_raw'] # 필요시 토픽 지정 가능
    )

    return LaunchDescription([
        camera_cmd,
        teleop_cmd,
        qr_node,
        rqt_node
    ])