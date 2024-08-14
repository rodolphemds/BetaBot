#!/bin/bash

begin()
{
echo "Running additionnal startup scripts"
}

reset_motors()
{
cd /betabot/shell_scripts/;
sudo ./motors_reset.sh;
}

launch_ros_betabot_core()
{
/opt/ros/melodic/bin/roslaunch /betabot/ros_launch/core.launch;
}

begin
reset_motors
launch_ros_betabot_core