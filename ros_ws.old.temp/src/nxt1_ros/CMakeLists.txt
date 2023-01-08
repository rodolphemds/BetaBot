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