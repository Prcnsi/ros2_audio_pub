from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_audio_pub',
            executable='mic_audio_with_doa',
            name='mic_audio_with_doa_node',
            output='screen'
        )
    ])
