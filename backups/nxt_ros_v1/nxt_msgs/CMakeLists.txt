cmake_minimum_required(VERSION 2.8.3)

project(nxt_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(FILES
  Accelerometer.msg
  ArmState.msg
  ArmCommand.msg
  Color.msg
  Contact.msg
  Gyro.msg
  HeadState.msg
  HeadCommand.msg
  JointCommand.msg
  LaserCommand.msg
  LaserState.msg
  Light.msg
  Range.msg
  RFID.msg
  Sound.msg
  TorsoState.msg
  TorsoCommand.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)