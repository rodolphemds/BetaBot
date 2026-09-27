from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joints_teleop',
            executable='joints_teleop.py',
            name='joints_teleop',
            output='screen',
        ),
    ])
