from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_enhanced_teleop',
            executable='base_enhanced_teleop.py',
            name='base_enhanced_teleop',
            output='screen',
        ),
    ])
