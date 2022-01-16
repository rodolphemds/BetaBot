#!/bin/bash

github_user_account="rodolphemds"
github_user_token="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" # this token will expire on 08/14/2022
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

download_betabot_filesystem()
{
echo "Cloning file system repository from GitHub..."
cd /betabot;
git clone https://${github_user_account}:${github_user_token}@github.com/${github_user_account}/${github_repository}.git;
# Making scripts executable
sudo chmod -R +x /betabot/shell_scripts/*;
sudo chmod -R +x /betabot/python_scripts/*;
# Allowing all users to read and modify files in the local copy of the repository
sudo chmod -R +rw /betabot/*;
echo "Cloning file system repository from GitHub... Done"
}

building_ros_packages()
{
echo "Building ROS packages...";
cd /betabot/ros_ws;
sudo catkin build;
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