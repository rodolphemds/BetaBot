cmake_minimum_required(VERSION 2.8.3)

project(nxt1_ros)

find_package(orocos_kdl REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  nxt_msgs

  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nxt_msgs rospy sensor_msgs
)

catkin_install_python(PROGRAMS
  src/nxt1_ros/nxt1_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})