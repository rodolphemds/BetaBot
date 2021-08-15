# /betabot/ros_launch
This folder contains all used launch files to start ROS packages. 

## RUNNING A LAUNCH FILE
In ROS launch files can be run with
```
roslaunch LaunchFile.launch
```

## LAUNCH FILES 
- core.launch: start all ROS core packages (nxts and opencr communication, webcam, lidar and state publisher)  publish the transforms and load the robot description.