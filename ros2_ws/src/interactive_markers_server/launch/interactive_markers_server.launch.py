from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interactive_markers_server',
            executable='interactive_markers_server.py',
            name='interactive_markers_server',
            output='screen',
        ),
    ])
