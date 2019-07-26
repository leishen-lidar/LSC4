cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c4_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarC4Layer.msg
  LslidarC4Packet.msg
  LslidarC4Point.msg
  LslidarC4Scan.msg
  LslidarC4Sweep.msg
  LslidarC4ScanUnified.msg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
