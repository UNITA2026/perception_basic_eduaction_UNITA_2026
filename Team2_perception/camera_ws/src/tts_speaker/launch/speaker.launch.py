from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tts_speaker')
    sound_path = os.path.join(pkg_share, 'sounds')

    return LaunchDescription([
        # ✅ 1) detections -> /red,/yellow,/green Bool 변환 브릿지 노드
        Node(
            package='tts_speaker',
            executable='traffic_light_bool_bridge',
            name='traffic_light_bool_bridge',
            output='screen',
            parameters=[
                {'detections_topic': '/detections'},
                {'hold_sec': 0.3},
            ]
        ),

        # ✅ 2) /red,/yellow,/green Bool 구독하여 mp3 재생 speaker
        Node(
            package='tts_speaker',
            executable='tts_speaker_node',
            name='speaker_node',
            output='screen',
            parameters=[
                {'sound_path': sound_path},
                # 필요시 옵션:
                # {'mode': 'edge'},
                # {'interrupt_on_change': True},
                # {'hold_sec': 0.5},
                # {'cooldown_sec': 0.8},
            ]
        )
    ])
