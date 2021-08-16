#!/bin/bash

## You can copy this file into /boot mounting point just after flashing the SD card and then after connecting to the Odroid C2 through ssh run it after second boot. On the Odroid C2 the boot partition is mounted on /media/boot.
## Please note that you must reboot once the odroid C2 before running this script.

## https://wiki.odroid.com/odroid-c2/os_images/ubuntu/v4.1

#####################
## SET PARAMETERS ##
#####################
robot_name="BetaBot"
ros_version="noetic" #ROS noetic requires the last Ubuntu 20.04 LTS release for Odroid C2
# standard_user_name="user" # must be modified manualy
# standard_user_password="password" # must be modified manualy
github_user_account="rodolphemds"
github_user_token="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" # this token will expire on 08/14/2022
github_repository="betabot"

#############################################
## OTHER USEFULL COMMANDS TO REMEMBER ##
#############################################

# Customize odroid user account and allow auto-login
#echo '### Changing default odroid user name to ${standard_user_name} with password ${standard_user_password}... ###'
#sudo usermod -l ${standard_user_name} odroid;
#sudo usermod -p ${standard_user_password} ${standard_user_name};
#sudo usermod -d /home/${standard_user_name} -m ${standard_user_name};
#rm -Rf /home/odroid;

#### Changing default odroid user name to ${standard_user_name} with password ${standard_user_password}... Done ###'
#sudo echo "### Configuring ${standard_user_name} auto-login... ###";
#sudo echo "[Seat:*]" > /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
#sudo echo "greeter-session=slick-greeter" >> /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
#sudo echo "autologin-user=${standard_user_name}" >> /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
#sudo echo "### Configuring ${standard_user_name} auto-login... Done ###";

##################
## BEGIN SCRIPT ##
##################
begin()
{
echo "### ... INITIAL SYSTEM SETUP SCRIPT ... ###";
echo "### Please note an internet connection is required. You must run this as the main standard user (ex. the default odroid account). The robot computer must be an Odroid C2 board with the default Ubuntu-Mate OS. ###" 
echo "### This script should only be run once after a first reboot. Press [Enter] to continue setup. ###";
read;
}

#################
## INSTALL ROS ##
#################
# Based on http://wiki.ros.org/noetic/Installation/Ubuntu
install_ros()
{
echo "### Installing ROS... ###";
# Installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list';
sudo apt install curl -y # if you haven't already installed curl;
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -;
sudo apt update;
sudo apt install ros-${ros_version}-desktop-full -y;
# Environment setup
echo "source /opt/ros/${ros_version}/setup.bash" >> ~/.bashrc;
source /opt/ros/${ros_version}/setup.bash;
# Dependencies for building packages
sudo apt install python3 python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y;
sudo rosdep init;
rosdep update;
# Install other packages
sudo apt-get install ros-${ros_version}-audio-common ros-${ros_version}-usb-cam ros-${ros_version}-rosserial-python ros-${ros_version}-tf ros-${ros_version}-joy ros-${ros_version}-teleop-twist-joy ros-${ros_version}-teleop-twist-keyboard ros-${ros_version}-laser-proc ros-${ros_version}-rgbd-launch ros-${ros_version}-depthimage-to-laserscan ros-${ros_version}-rosserial-arduino ros-${ros_version}-rosserial-python ros-${ros_version}-rosserial-server ros-${ros_version}-rosserial-client ros-${ros_version}-rosserial-msgs ros-${ros_version}-amcl ros-${ros_version}-map-server ros-${ros_version}-move-base ros-${ros_version}-urdf ros-${ros_version}-xacro ros-${ros_version}-compressed-image-transport ros-${ros_version}-rqt-image-view ros-${ros_version}-gmapping ros-${ros_version}-navigation ros-${ros_version}-interactive-markers ros-${ros_version}-rosserial-python ros-${ros_version}-tf -y
python3 -m pip install pysub;
#  Set environment variables
echo "export ROS_IP=localhost" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
# End
echo "### Installing ROS... Done ###";
}

######################
## INSTALL PACKAGES ##
######################
install_packages()
{
echo "### Updating and installing other usefull packages... ###";
sudo add-apt-repository ppa:hardkernel/ppa -y;
sudo apt-get update;
sudo apt-get install odroid-wiringpi-python software-properties-common odroid-wiringpi libwiringpi-dev libwiringpi2 usbutils wget git alsa-utils xterm unzip software-properties-common firefox cheese xz-utils tar tightvncserver locate blueman streamer smbclient chrony ntpdate -y;
# End
echo "### Updating and installing other usefull packages... Done ###";
}

################################
## CLONE BETABOT REPOSITORY ##
################################
install_filesystem()
{
echo "### Cloning file system repository from GitHub and building ROS packages... ###";
# Downloading repository
cd /
sudo git clone https://${github_user_account}:${github_user_token}@github.com/${github_user_account}/${github_repository}.git;
# Making scripts executable
sudo chmod +x /betabotbot/shell_scripts/*;
sudo chmod +x /betabotbot/python_scripts/*;
# Sourcing shell scripts
echo "export PATH=/betabot/shell_scripts/:$PATH" >> ~/.bashrc;
# Adding swap
echo "Adding swap...";
sudo dd if=/dev/zero of=/swapfile bs=64M count=16;
sudo mkswap /swapfile;
sudo swapon /swapfile;
echo "Adding swap...Done";
# Building ROS packages
echo "Building ROS packages...";
cd /betabot/ros_ws;
catkin build;
echo "source /betabot/ros_ws/devel/setup.bash">> ~/.bashrc;
echo "Building ROS packages... Done";
# End
echo "### Cloning file system repository from GitHub... Done ###";
}

##################################
## INSTALL STEREO BOOM BONNET ##
##################################
install_sound()
{
echo "### Installing stereo sound bonnet... ###"
# Load kernel modules at boot
sudo modprobe snd-soc-pcm5102;
sudo modprobe snd-soc-odroid-dac;
sudo bash -c 'echo "snd-soc-pcm5102" >> /etc/modules';
sudo bash -c 'echo "snd-soc-odroid-dac" >> /etc/modules';
# Set default speaker
echo set-default-sink 0 | sudo tee -a /etc/pulse/default.pa;
# End
echo "### Installing stereo sound bonnet... Done ###"
}

####################################
## OTHER SYSTEM CONFIGURATIONS ##
####################################
system_config()
{
# Update date and time
echo "### Updating date and time... ###";
sudo ntpdate ntp.ubuntu.com;
echo "### Updating date and time... Done ###";

# Disable root account
echo "### Disabling root account... ###";
sudo passwd -l root;
echo "### Disabling root account... Done ###";

# Define robot name
echo "### Change robot name (default = ${robot_name})... ###";
sudo sed -i "s/odroid/${robot_name}/g" /etc/hostname;
sudo sed -i "s/odroid64/${robot_name}/g" /etc/hosts; #https://www.cyberciti.biz/faq/how-to-use-sed-to-find-and-replace-text-in-files-in-linux-unix-shell/
echo "### Change robot name (default = ${robot_name})... Done ###";

# Configure distant file access
echo "### Configuring samba share... ###";
sudo apt-get install samba -y;
sudo touch /etc/libuser.conf;
sudo bash -c 'echo "[sambashare]" >> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t comment = Robot HD samba share">> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t path = /">> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t read only = no" >> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t writeable = yes" >> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t browsable = yes" >> /etc/samba/smb.conf';
sudo bash -c 'echo -e "/t guest ok = yes" >> /etc/samba/smb.conf';
sudo ufw allow samba;
echo "Guest access has been authorized, you will not need any password to connect to the robot samba share";
echo "### Configuring samba share... Done ###";
 
 # Run commands listed on /betabot/shell_scripts/on_shutdown.sh to run at shutdown
echo "### Configuring scripts running on shutdown... ###";
sudo bash -c 'echo "[Unit]" > /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "Description=Run shutdown additional commands from /betabot/shell_scripts/on_shutdown.sh" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "[Service]" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "Type=oneshot" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "ExecStop=/betabot/shell_scripts/on_shutdown.sh" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "RemainAfterExit=yes" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "[Install]" >> /etc/systemd/system/shutdown-scripts.service';
sudo bash -c 'echo "WantedBy=multi-user.target" >> /etc/systemd/system/shutdown-scripts.service'
sudo systemctl enable shutdown-scripts.service;
echo "### Configuring scripts running on shutdown... Done ###";

 # Run commands listed on /betabot/shell_scripts/on_startup.sh to run at startup
echo "### Configuring scripts running on startup... ###";
sudo bash -c 'echo "[Unit]" > /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "Description=Run startup additional commands from /betabot/shell_scripts/on_startup.sh" >> /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "[Service]" >> /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "Type=simple" >> /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "ExecStop=//betabot/shell_scripts/on_startup.sh" >> /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "[Install]" >> /etc/systemd/system/startup-scripts.service';
sudo bash -c 'echo "WantedBy=multi-user.target" >> /etc/systemd/system/startup-scripts.service
sudo systemctl enable startup-scripts.service';
echo "### Configuring scripts running on startup... Done ###";

# Configuring message appearing on ssh connection
echo "### Configuring the motd message... ###";
# chmod -x /etc/update-motd.d/10-help-text;
sudo bash -c 'echo 'printf "--- ROBOT COMMAND LINE INTERFACE ---"' >> /etc/update-motd.d/00-title';
chmod +x /etc/update-motd.d/00-title;
echo "### Configuring the motd message... Done ###";

# Ask user to configure the keyboard
echo "### Configuring keyboard layout... ###";
sudo dpkg-reconfigure keyboard-configuration;
echo "### Configuring keyboard layout... Done ###";

# Ask user to set default text editor
echo "### Configuring default text editor... ###";
update-alternatives --config editor;
echo "### Configuring default text editor... Done ###";

# Ask user to set VNC password
echo "Please choose a password to for VNC connection...";
vncpasswd;
echo "VNC password defined";
}

##################
## SCRIPT 'S END##
##################
end()
{
# Cleaning and end
sudo apt-get autoremove -y;
sudo apt-get autoclean -y;
echo "### Script finished. Please restart the robot. ###";
echo "### Do not forget to read this script which contains usefull commands to set more settings. ###";
echo "### Please update the OS using the script in /betabot/shell_scripts/update_os.sh. ###";
exit
}




begin
install_ros
install_packages
install_filesystem
install_sound
system_config
end

