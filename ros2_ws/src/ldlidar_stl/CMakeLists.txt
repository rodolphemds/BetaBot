cmake_minimum_required(VERSION 3.8)
project(ldlidar_stl_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/ldlidar_driver/include
)

add_executable(${PROJECT_NAME}_node src/publish_node/main.cpp
  ldlidar_driver/src/core/ldlidar_driver.cpp
  ldlidar_driver/src/dataprocess/lipkg.cpp
  ldlidar_driver/src/filter/tofbf.cpp
  ldlidar_driver/src/logger/log_module.cpp
  ldlidar_driver/src/networkcom/network_socket_interface_linux.cpp
  ldlidar_driver/src/serialcom/serial_interface_linux.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)

add_executable(${PROJECT_NAME}_listen_node src/listen_node/listen_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_listen_node rclcpp sensor_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_listen_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
