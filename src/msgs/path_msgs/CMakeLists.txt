cmake_minimum_required(VERSION 2.8.3)
project(path_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Node.msg
  Link.msg
  Cost.msg
  Bezier.msg
  Trajectory.msg
  Route.msg
  Waypoint.msg
  Map.msg
  Lane.msg
  State.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime)
