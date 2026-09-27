cmake_minimum_required(VERSION 2.8.3)

project(emergency_shutdown)

find_package(orocos_kdl REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS
  src/emergency_shutdown/emergency_shutdown.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})