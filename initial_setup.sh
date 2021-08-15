#!/bin/bash

## You can copy this file into /boot mounting point just after flashing the SD card and then run it after second boot. To make this script executable run chmod +x ThisScript

#####################
## SET PARAMETERS ##
#####################
robot_name="BetaBot"
ros_version="noetic" #ROS noetic requires the last Ubuntu 20.04 LTS release for Odroid C2
standard_user_name="user"
standard_user_password="password"
github_user_account="rodolphemds"
github_user_token="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" # this token will expire on 08/14/2022
github_repository="betabot"

##################
## BEGIN SCRIPT ##
##################
begin()
{
echo "### ... INITIAL SYSTEM SETUP SCRIPT ... ###";
echo "Please note an internet connection is required. You must run this as the main standard user (ex. the default odroid account). The robot computer must be an Odroid c2 board with the default Ubuntu-Mate OS." 
echo "This script should only be run once after a first reboot. Press [Enter] to continue setup.";
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
# Dependencies for building packages
sudo apt install python-rosdep python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential  python-catkin-tools -y;
sudo rosdep init;
rosdep update;
# Install other packages
sudo apt-get install ros-${ros-version}-audio-common ros-${ros-version}-usb-cam ros-${ros-version}-rosserial-python ros-${ros-version}-tf ros-${ros-version}-joy ros-${ros-version}-teleop-twist-joy ros-${ros-version}-teleop-twist-keyboard ros-${ros-version}-laser-proc ros-${ros-version}-rgbd-launch ros-${ros-version}-depthimage-to-laserscan ros-${ros-version}-rosserial-arduino ros-${ros-version}-rosserial-python ros-${ros-version}-rosserial-server ros-${ros-version}-rosserial-client ros-${ros-version}-rosserial-msgs ros-${ros-version}-amcl ros-${ros-version}-map-server ros-${ros-version}-move-base ros-${ros-version}-urdf ros-${ros-version}-xacro ros-${ros-version}-compressed-image-transport ros-${ros-version}-rqt-image-view ros-${ros-version}-gmapping ros-${ros-version}-navigation ros-${ros-version}-interactive-markers ros-${ros-version}-rosserial-python ros-${ros-version}-tf -y
python2 -m pip install pyusb;
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
echo "### Installing other usefull packages... ###";
sudo add-apt-repository ppa:hardkernel/ppa;
sudo apt-get update;
sudo apt-get upgrade -y;
sudo apt-get dist-upgrade -y;
sudo apt-get install libnfs11 libcec odroid-wiringpi-python software-properties-common odroid-wiringpi libwiringpi-dev libwiringpi2 usbutils wget git alsa-utils xterm unzip software-properties-common firefox cheese xz-utils tar tightvncserver python3-pip locate python3 blueman streamer python-pip smbclient samba system-config-samba chrony ntpdate -y;
sudo apt-get autoremove -y;
sudo apt-get autoclean -y;
# End
echo "### Installing other usefull packages... Done ###";
}

################################
## CLONE BETABOT REPOSITORY ##
################################
install_filesystem()
{
echo "### Cloning file system repository from GitHub... ###";
# Downloading repository
cd /
sudo git clone https://${github_user_account}:${github_user_token}@github.com/${github_user_account}/${github_repository}.git;
# Making scripts executable
sudo chmod+x /betabotbot/shell_scripts/*;
sudo chmod+x /betabotbot/python_scripts/*;
# Sourcing shell scripts
echo "source /betabot/shell_scripts/" >> ~/.bashrc;
# Adding swap
echo "Adding swap...";
dd if=/dev/zero of=/swapfile bs=64M count=16;
mkswap /swapfile;
swapon /swapfile;
echo "Adding swap...Done";
# Building ROS packages
cd /betabot/ros_ws;
catkin build;
echo "source /betabot/ros_ws/devel/setup.bash">> ~/.bashrc;
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
echo "snd-soc-pcm5102" >> /etc/modules
echo "snd-soc-odroid-dac" >> /etc/modules
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
ntpdate ntp.ubuntu.com;
echo "### Updating date and time... Done ###";

# Disable root account
echo "### Disabling root account... ###";
sudo passwd -l root;
echo "### Disabling root account... Done ###";

# Customize odroid user account and allow auto-login
echo '### Changing default "odroid" user name to "${standard_user_name}" with password "${standard_user_password}"... ###'
usermod -l ${standard_user_name} odroid;
usermod -p ${standard_user_password} ${standard_user_name};
usermod -d /home/${standard_user_name} -m ${standard_user_name};
rm -R /home/odroid;
echo '### Changing default "odroid" user name to "${standard_user_name}" with password "${standard_user_password}"... Done ###';
echo "### Configuring ${standard_user_name} auto-login... ###";
echo "[Seat:*]" > /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
echo "greeter-session=slick-greeter" >> /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
echo "autologin-user=${standard_user_name}" >> /usr/share/lightdm/lightdm.conf.d/50-slick-greeter.conf;
echo "### Configuring ${standard_user_name} auto-login... Done ###";

# Define robot name
echo "### Change robot name (default = ${robot_name})... ###";
echo "${robot_name}" > /etc/hostname;
sed -i "s/odroid64/${robot_name}/g" /etc/hosts; #https://www.cyberciti.biz/faq/how-to-use-sed-to-find-and-replace-text-in-files-in-linux-unix-shell/
echo "### Change robot name (default = ${robot_name})... Done ###";

# Configure distant file access
echo "### Configuring samba share... ###";
touch /etc/libuser.conf;
echo '[sambashare]' >> /etc/samba/smb.conf;
echo -e '/t comment = Robot HD samba share' >> /etc/samba/smb.conf;
echo -e '/t path = /' >> /etc/samba/smb.conf;
echo -e '/t read only = no' >> /etc/samba/smb.conf;
echo -e '/t writeable = yes' >> /etc/samba/smb.conf;
echo -e '/t browsable = yes' >> /etc/samba/smb.conf;
echo -e '/t valid users = ${standard_user_name} root' >> /etc/samba/smb.conf;
sudo ufw allow samba;
echo "### Configuring samba share... Done ###";
 
 # Run commands listed on /betabot/shell_scripts/on_shutdown.sh to run at shutdown
echo "### Configuring scripts running on shutdown... ###";
echo '[Unit]' > /etc/systemd/system/shutdown-scripts.service;
echo 'Description=Run shutdown additional commands from /betabot/shell_scripts/on_shutdown.sh' >> /etc/systemd/system/shutdown-scripts.service;
echo '[Service]' >> /etc/systemd/system/shutdown-scripts.service;
echo 'Type=oneshot' >> /etc/systemd/system/shutdown-scripts.service;
echo 'ExecStop=/betabot/shell_scripts/on_shutdown.sh' >> /etc/systemd/system/shutdown-scripts.service;
echo 'RemainAfterExit=yes' >> /etc/systemd/system/shutdown-scripts.service;
echo '[Install]' >> /etc/systemd/system/shutdown-scripts.service;
echo 'WantedBy=multi-user.target' >> /etc/systemd/system/shutdown-scripts.service
systemctl enable shutdown-scripts.service;
echo "### Configuring scripts running on shutdown... Done ###";

 # Run commands listed on /betabot/shell_scripts/on_startup.sh to run at startup
echo "### Configuring scripts running on startup... ###";
echo '[Unit]' > /etc/systemd/system/startup-scripts.service;
echo 'Description=Run startup additional commands from /betabot/shell_scripts/on_startup.sh' >> /etc/systemd/system/startup-scripts.service;
echo '[Service]' >> /etc/systemd/system/startup-scripts.service;
echo 'Type=simple' >> /etc/systemd/system/startup-scripts.service;
echo 'ExecStop=//betabot/shell_scripts/on_startup.sh' >> /etc/systemd/system/startup-scripts.service;
echo '[Install]' >> /etc/systemd/system/startup-scripts.service;
echo 'WantedBy=multi-user.target' >> /etc/systemd/system/startup-scripts.service
systemctl enable startup-scripts.service;
echo "### Configuring scripts running on startup... Done ###";
}

##################
## SCRIPT 'S END##
##################
end()
{
echo "### Script finished. Please restart the robot. ###"
exit
}

#############################################
## OTHER USEFULL COMMANDS TO REMEMBER ##
#############################################
#echo "Configure keyboard layout...";
#dpkg-reconfigure keyboard-configuration;
#echo "Configure default text editor...";
#update-alternatives --config editor;
# echo "Configuring the motd message...";
# chmod -x /etc/update-motd.d/10-help-text;
# echo 'printf "--- robot command line interface ---"' >> /etc/update-motd.d/00-title;
# chmod +x /etc/update-motd.d/00-title;
# echo "Configure VNC user password...";
# vncpasswd;




begin
install_ros
install_packages
install_filesystem
install_sound
system_config
end

