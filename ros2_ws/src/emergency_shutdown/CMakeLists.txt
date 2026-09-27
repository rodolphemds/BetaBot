cmake_minimum_required(VERSION 3.8)
project(emergency_shutdown)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/emergency_shutdown.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
