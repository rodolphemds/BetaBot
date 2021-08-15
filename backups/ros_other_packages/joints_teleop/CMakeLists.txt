cmake_minimum_required(VERSION 2.8.3)

project(joints_teleop)

find_package(orocos_kdl REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  nxt_msgs
  nxt_python
  rospy
  sensor_msgs
  nxt1_ros
  nxt2_ros
  nxt_controllers
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nxt_msgs nxt_python rospy sensor_msgs nxt1_ros nxt2_ros nxt_controllers
)

catkin_install_python(PROGRAMS
  src/joints_teleop/joints_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)