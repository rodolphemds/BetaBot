# /betabot/ros_launch
This folder contains all used launch files to start ROS packages. 

## RUNNING A LAUNCH FILE
In ROS launch files can be run with
```
roslaunch LaunchFile.launch
```

## LAUNCH FILES 
- core.launch: start all ROS core packages (nxts and opencr communication, webcam, lidar and state publisher)  publish the transforms and load the robot description.
- slam.launch: start gmapping to build a map from the lidar scan and the treads odometry.
- teleop_joy.launch: remote control the base with a joystick (joy + teleop_twist_joy).
- rviz_core.launch: start the core and rviz for visualisation.
- base_simple_teleop.launch / base_enhanced_teleop.launch: keyboard control of the base.
- joints_teleop.launch: keyboard control of head, torso, laser and arms joints.