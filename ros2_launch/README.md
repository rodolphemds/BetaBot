# /betabot/ros2_launch

This folder contains all used launch files to start the ROS 2 packages.

## RUNNING A LAUNCH FILE

In ROS 2, launch files can be run with

```
ros2 launch <package_name> <launch_file.launch.py>
```

or by giving the full path:

```
ros2 launch /betabot/ros2_launch/core.launch.py
```

## LAUNCH FILES

- core.launch.py: start all core ROS 2 packages (nxts communication, webcam, lidar and
  state publisher), publish the transforms and load the robot description.
- rviz_core.launch.py: start core.launch.py and rviz2.
- base_enhanced_teleop.launch.py: keyboard teleoperation of the base (curses).
- base_simple_teleop.launch.py: simple arrow-keys teleoperation of the base.
- joints_teleop.launch.py: keyboard control of the head, torso, laser and arms joints.
