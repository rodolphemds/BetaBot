# /betabot/
This folder is the main folder containing all BetaBot specific folder and files. 
Each folder contains a README.md file explaining what is the folder about.

NB : As this folder is not intended to be modified during robot work, all temporary or user created files such as recorded maps are not stored in this folder but in the user's home space.

## FIRST BOOT
Reboot once the ODROID after flashing before running the script at the second boot.
Copy the initial_setup.sh file, connect through SSH and run :
```
sudo ./initial_setup.sh
```
You can copy this file into /boot mounting point just after flashing the SD card.
You can make adjustment by setting variables at the beginning of this script. 
Do not forget to read the script wich contains usefull command to set other parameters.

## BACKUPS
This folder contains older versions of BetBot software work.

## DESCRIPTION
This folder contains the urdf ahd mesh files which describes BetaBot hardware to be used by ROS2.

## DOCS 
This folder contains usefull documentation for BetaBot's hardware and software.

## MEDIA
This folder contains sounds, pictures, icons and other multimedia ressources for BetaBot.

## PYTHON_SCRIPTS
This folder contains usefull python scripts created for the BetaBot.

## ROS_LAUNCH
This folder contains the ROS launch files.

## ROS_WS
This folder is the ROS catkin workspace.

##SHELL_SCRIPTS
This folder contains usefull bash scripts created for the BetaBot.