cmake_minimum_required(VERSION 2.8.3)
project(localization_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

add_message_files(
    FILES
    node.msg
    lidar.msg
    pole.msg
    poles.msg
    planar.msg
    planars.msg
    mcl.msg
    ObjTrackBox.msg
    ObjTrackBoxes.msg
    FeatureTrackBox.msg
    FeatureTrackBoxes.msg
    PoleTrackBox.msg
    PoleTrackBoxes.msg
    PlanarTrackBox.msg
    PlanarTrackBoxes.msg
    FeatureBoxes.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
   )

catkin_package(
CATKIN_DEPENDS message_runtime)
