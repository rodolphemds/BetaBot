#!/bin/bash

github_user_account="rodolphemds"
github_user_token="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" # this token will expire on 08/14/2022
github_repository="betabot"


begin()
{
echo "Updating file system..."
}

remove_old_filesystem()
{
sudo rm -rf /betabot;
}

download_betabot_filesystem()
{
cd /betabot;
git clone https://${github_user_account}:${github_user_token}@github.com/${github_user_account}/${github_repository}.git;
}

building_ros_packages()
{
cd /betabot/ros_ws;
catkin build;
source /betabot/ros_ws/devel/setup.bash;
}

source_scripts()
{
source /betabot/shell_scripts/
sudo chmod+x /betabotbot/shell_scripts/*
sudo chmod+x /betabotbot/python_scripts/*

end()
{
echo "File system is up-to-date."
exit
}

begin
remove_old_filesystem
download_new_filesystem
building_ros_packages
source_scripts
end