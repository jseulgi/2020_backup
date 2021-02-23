cmake_minimum_required(VERSION 3.0.2)
project(obj_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_message_files(
  FILES
  GridRoi.msg
  Ego.msg
  Obj.msg
  ObjList.msg
  ObjTrackBox.msg
  ObjTrackBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
