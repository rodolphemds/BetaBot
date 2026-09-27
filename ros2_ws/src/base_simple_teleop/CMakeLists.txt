cmake_minimum_required(VERSION 3.8)
project(base_simple_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER MATCHES "clan" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/base_simple_teleop.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
