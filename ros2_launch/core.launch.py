import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    urdf_path = '/betabot/description/urdf/robot_description.urdf'
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
        robot_description_param = {'robot_description': robot_description}
    else:
        robot_description_param = {'robot_description': ''}

    return LaunchDescription([
        # robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='transforms_publisher',
            output='screen',
            parameters=[robot_description_param, {'publish_frequency': 10.0}],
        ),

        # connection to NXT1
        Node(
            package='nxt1_ros',
            executable='nxt1_ros',
            name='nxt1_ros',
            output='screen',
            respawn=True,
        ),

        # connection to NXT2
        Node(
            package='nxt2_ros',
            executable='nxt2_ros',
            name='nxt2_ros',
            output='screen',
            respawn=True,
        ),

        # joints_state publisher
        Node(
            package='nxt_controllers',
            executable='joint_states_aggregator',
            name='joint_states_publisher',
            output='screen',
            respawn=True,
        ),

        # base controller
        Node(
            package='nxt_controllers',
            executable='base_controller',
            name='base_controller',
            output='screen',
            respawn=True,
        ),

        # treads odometry
        Node(
            package='nxt_controllers',
            executable='base_odometry',
            name='odom_publisher',
            output='screen',
            respawn=True,
        ),

        # joints controller
        Node(
            package='nxt_controllers',
            executable='joints_controller',
            name='head_joint_controller',
            output='screen',
            respawn=True,
            parameters=[{'name': 'head_joint'}],
        ),
        Node(
            package='nxt_controllers',
            executable='joints_controller',
            name='laser_joint_controller',
            output='screen',
            respawn=True,
            parameters=[{'name': 'laser_joint'}],
        ),
        Node(
            package='nxt_controllers',
            executable='joints_controller',
            name='arms_joint_controller',
            output='screen',
            respawn=True,
            parameters=[{'name': 'arms_joint'}],
        ),
        Node(
            package='nxt_controllers',
            executable='joints_controller',
            name='torso_joint_controller',
            output='screen',
            respawn=True,
            parameters=[{'name': 'torso_joint'}],
        ),

        # fixed transform for base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_fixed_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),

        # connection to webcam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='head_camera',
            output='screen',
            respawn=True,
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'framerate': 10.0,
                'camera_name': 'head_camera',
                'camera_frame_id': 'usb_cam_link',
                'io_method': 'mmap',
            }],
        ),

        # connection to lidar
        Node(
            package='ldlidar_stl_ros',
            executable='ldlidar_stl_ros_node',
            name='lidar',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'topic_name': 'scan',
                'frame_id': 'base_laser',
                'port_name': '/dev/ttyS1',
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 0.0,
            }],
        ),

        # emergency shutdown node
        Node(
            package='emergency_shutdown',
            executable='emergency_shutdown.py',
            name='emergency_shutdown_request',
            output='screen',
            respawn=True,
        ),
    ])
