cmake_minimum_required(VERSION 3.8)
project(point_operation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/point_operation.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
