from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_simple_teleop',
            executable='base_simple_teleop_node',
            name='base_simple_teleop',
            output='screen',
            parameters=[{'scale_linear': 1.0, 'scale_angular': 1.0}],
        ),
    ])
