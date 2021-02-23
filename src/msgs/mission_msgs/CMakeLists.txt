cmake_minimum_required(VERSION 3.0.2)
project(mission_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

add_message_files(
  FILES
  mission.msg
  mission_list.msg
  global2mission.msg
  global2mission_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)
