import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # =========================================================
    # 1. [심소진 패키지 실행] Smart Hospital System 
    # =========================================================
    
    # 패키지 경로 찾기
    hospital_pkg_name = 'smart_hospital_system'
    hospital_pkg_dir = get_package_share_directory(hospital_pkg_name)
    
    # 런치 파일 경로 설정
    hospital_launch_file = os.path.join(hospital_pkg_dir, 'launch', 'hospital_system.launch.py')

    # 실행 객체 생성 (Include)
    hospital_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hospital_launch_file)
    )

    # =========================================================
    # 2. [팀원 패키지 설정] 나중에 패키지/노드 이름만 수정하세요!
    # =========================================================
    
    # 2-1. GUI 팀 정보
    GUI_PKG_NAME  = 'unknown_gui_pkg'   # 나중에 실제 패키지 명으로 수정
    GUI_EXEC_NAME = 'unknown_gui_node'  # 나중에 실제 실행 파일 명으로 수정

    # 2-2. TTS 팀 정보
    TTS_PKG_NAME  = 'unknown_tts_pkg'   # 나중에 실제 패키지 명으로 수정
    TTS_EXEC_NAME = 'unknown_tts_node'  # 나중에 실제 실행 파일 명으로 수정

    # =========================================================
    # 3. [팀원 노드 정의] 수정할 필요 없습니다.
    # =========================================================

    gui_node = Node(
        package    = GUI_PKG_NAME,
        executable = GUI_EXEC_NAME,
        name       = 'gui_node',
        output     = 'screen',
    )

    tts_node = Node(
        package    = TTS_PKG_NAME,
        executable = TTS_EXEC_NAME,
        name       = 'tts_node',
        output     = 'screen'
    )

    # =========================================================
    # 4. [최종 실행 목록] LaunchDescription 리턴
    # =========================================================
    
    ld = LaunchDescription()

    # (1) 내 병원 시스템은 무조건 실행
    ld.add_action(hospital_system_launch)

    # (2) 팀원들 노드는 패키지가 존재할 때만 주석을 풀어서 사용하세요!
    # 지금 'unknown_pkg'인 상태로 아래 주석을 풀면 에러가 나서 내꺼도 실행이 안 됩니다.
    
    # ld.add_action(gui_node)  # <--- 패키지 받으면 주석 해제
    # ld.add_action(tts_node)  # <--- 패키지 받으면 주석 해제

    return ld
