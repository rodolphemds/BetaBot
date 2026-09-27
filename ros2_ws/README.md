# /betabot/ros2_ws

ROS 2 (colcon) workspace containing the BetaBot packages converted from ROS 1.

## BUILD

```bash
cd /betabot/ros2_ws
source /opt/ros/<distro>/setup.bash   # e.g. humble or jazzy
colcon build --symlink-install
source install/setup.bash
```

## PACKAGES

- `nxt_msgs`: NXT sensor and motor messages (rosidl).
- `nxt_python`: NXT-python library dependency declaration.
- `nxt1_ros`: bindings between the NXT1 brick and ROS 2 (rclpy).
- `nxt2_ros`: bindings between the NXT2 brick and ROS 2 (rclpy).
- `nxt_controllers`: base controller, base odometry, joints controller and
  joint states aggregator (rclpy).
- `base_enhanced_teleop`: keyboard teleoperation of the base (curses).
- `base_simple_teleop`: simple arrow-keys teleoperation of the base (rclcpp).
- `joints_teleop`: keyboard control of the head, torso, laser and arms joints.
- `emergency_shutdown`: emergency shutdown node.
- `point_operation`: drive the robot to an inserted x/y/z goal.
- `interactive_markers_server`: rviz interactive marker control of the base.
- `ldlidar_stl_ros`: LDLiDAR LD06/LD19 driver node (rclcpp).

## NOTES

- The original ROS 1 (catkin) workspace is kept in `/betabot/ros_ws`.
- Launch files are available in each package `launch/` folder and in
  `/betabot/ros2_launch`.
- The `robot_pose_ekf` and `rosserial` (OpenCR) nodes from the ROS 1 stack have no
  direct ROS 2 equivalent here; `robot_localization` and a `micro_ros` agent are the
  recommended replacements.
