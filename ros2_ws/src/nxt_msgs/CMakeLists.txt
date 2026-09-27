cmake_minimum_required(VERSION 3.8)
project(nxt_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Color.msg"
  "msg/Contact.msg"
  "msg/JointCommand.msg"
  "msg/Light.msg"
  "msg/RFID.msg"
  "msg/Sound.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
