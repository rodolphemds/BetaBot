# /betabot/backups/nxt_programming
This folder contains software needed to be downloaded on the Mindstorms NXT bricks. 

## MINDSTORMS NXT FIRMWARE
LEGO MINDSTORMS NXT Firmware V1.31.rfw is the actual and last firmware downloaded into the two Mindstorms NXT bricks. 

## .RBT FILES
.rbt file are created using the NXT-G graphical programming environment developped by Lego and LabVIEW.

1/ NXT1_calibrate.rbt and NXT2_calibrate.rbt are used to reset motors default position.
2/ NXT2_rfid_read.rbt is used as a workarround to allow rfid data to be used with ROS through the NXT ROS python packages. It must be run continously on the NXT2 brick.
3/ NXT2_light_anim.rbt is a small program to make a short light animation on the NXT2 color sensor.

## CONFIGURATION FILES
NVConfig.sys and RPGReader.sys are two configuration files extracted from a NXT memory configurated to prevent automatic sleep.
