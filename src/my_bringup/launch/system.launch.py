import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # [설정 영역] 나중에 팀원들에게 받아서 이 부분만 수정하세요!
    # ==========================================
    
    # 1. GUI 팀 정보
    GUI_PKG_NAME  = 'unknown_gui_pkg'   # 예: 'face_gui'
    GUI_EXEC_NAME = 'unknown_gui_node'  # 예: 'gui_main'

    # 2. TTS 팀 정보
    TTS_PKG_NAME  = 'unknown_tts_pkg'   # 예: 'voice_tts'
    TTS_EXEC_NAME = 'unknown_tts_node'  # 예: 'tts_main'
    
    # ==========================================
    # [실행 영역] 아래 코드는 수정할 필요 없습니다.
    # ==========================================

    # GUI 노드 정의 (이름이 틀려도 실행 시도하도록 설정)
    gui_node = Node(
        package    = GUI_PKG_NAME,
        executable = GUI_EXEC_NAME,
        name       = 'gui_node',
        output     = 'screen',
        # 패키지가 없어도 일단 런치 파일이 죽지 않고 경고만 뜨게 하려면 아래 옵션 고려 가능
        # condition=IfCondition(...) 복잡해지니 일단 생략
    )

    # TTS 노드 정의
    tts_node = Node(
        package    = TTS_PKG_NAME,
        executable = TTS_EXEC_NAME,
        name       = 'tts_node',
        output     = 'screen'
    )

    return LaunchDescription([
        gui_node,
        tts_node
    ])