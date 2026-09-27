cmake_minimum_required(VERSION 2.8.3)

project(nxt_controllers)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  nxt_msgs
  rospy
  sensor_msgs
  tf
  tf_conversions
)

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nav_msgs nxt_msgs rospy sensor_msgs tf tf_conversions
)

catkin_install_python(PROGRAMS
  src/nxt_controllers/base_controller.py
  src/nxt_controllers/base_odometry.py
  src/nxt_controllers/joints_controller.py
  src/nxt_controllers/joint_states_aggregator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
