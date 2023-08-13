cmake_minimum_required(VERSION 3.1)
project(hydra_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation voxblox_msgs)

add_message_files(
  FILES
  ActiveLayer.msg
  ActiveMesh.msg
  DsgUpdate.msg
  ObjectInfo.msg
)

add_service_files(FILES GetDsg.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs voxblox_msgs)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  voxblox_msgs
  message_runtime
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)
