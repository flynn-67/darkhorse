from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # QR API 서버
        Node(
            package='smart_hospital_system',
            executable='qr_api_node',
            output='screen'
        ),

        # 환자 UI
        Node(
            package='smart_hospital_system',
            executable='patient_ui',
            output='screen'
        ),

        # 의사 UI
        Node(
            package='smart_hospital_system',
            executable='doctor_ui',
            output='screen'
        ),

        # 대시보드 UI
        Node(
            package='smart_hospital_system',
            executable='dashboard_ui',
            output='screen'
        ),
    ])

