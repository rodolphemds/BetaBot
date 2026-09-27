cmake_minimum_required(VERSION 3.8)
project(joints_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/joints_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
