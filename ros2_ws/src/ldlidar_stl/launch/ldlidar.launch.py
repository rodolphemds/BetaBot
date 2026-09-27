from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_LD06'),
        DeclareLaunchArgument('topic_name', default_value='scan'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('port_baudrate', default_value='230400'),
        DeclareLaunchArgument('laser_scan_dir', default_value='true'),
        DeclareLaunchArgument('enable_angle_crop_func', default_value='false'),
        DeclareLaunchArgument('angle_crop_min', default_value='0.0'),
        DeclareLaunchArgument('angle_crop_max', default_value='0.0'),
        Node(
            package='ldlidar_stl_ros',
            executable='ldlidar_stl_ros_node',
            name='LDLiDAR',
            output='screen',
            parameters=[{
                'product_name': LaunchConfiguration('product_name'),
                'topic_name': LaunchConfiguration('topic_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'port_name': LaunchConfiguration('port_name'),
                'port_baudrate': LaunchConfiguration('port_baudrate'),
                'laser_scan_dir': LaunchConfiguration('laser_scan_dir'),
                'enable_angle_crop_func': LaunchConfiguration('enable_angle_crop_func'),
                'angle_crop_min': LaunchConfiguration('angle_crop_min'),
                'angle_crop_max': LaunchConfiguration('angle_crop_max'),
            }],
        ),
    ])
