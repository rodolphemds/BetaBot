from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_operation',
            executable='point_operation.py',
            name='point_operation',
            output='screen',
        ),
    ])
