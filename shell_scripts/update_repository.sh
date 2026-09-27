#!/bin/bash

github_user_account="rodolphemds"
github_repository="betabot"


begin()
{
echo "### Updating local copy of the core robot repository..."
}

remove_old_filesystem()
{
echo "Erasing old local copy of the repository..."
sudo rm -rf /betabot;
echo "Erasing old local copy of the repository... Done"
}

download_new_filesystem()
{
echo "Cloning repository from GitHub..."
cd /;
git clone https://${github_user_account}@github.com/${github_user_account}/${github_repository}.git;
# Making scripts executable
sudo chmod -R +x /betabot/shell_scripts/*;
sudo chmod -R +x /betabot/python_scripts/*;
# Allowing all users to read and modify files in the local copy of the repository
sudo chmod -R +rw /betabot/*;
echo "Cloning repository from GitHub... Done"
}

building_ros_packages()
{
echo "Building ROS packages...";
cd /betabot/ros_ws;
catkin_make;
# sudo catkin build;
source /betabot/ros_ws/devel/setup.bash;
echo "Building ROS packages... Done";
}

end()
{
echo "### Updating local copy of the core robot repository... Done"
exit
}

begin
remove_old_filesystem
download_new_filesystem
building_ros_packages
end