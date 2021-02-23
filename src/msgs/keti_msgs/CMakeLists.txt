cmake_minimum_required(VERSION 2.8.3)
project(keti_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
   FILES
   EPS.msg
   ACC.msg
   MoConf.msg
   MoVal.msg
   Point.msg
   dtlane.msg
   dtlaneArray.msg
   Node.msg
   JNode.msg
   JNodeArray.msg
   StopLine.msg
   StopLineArray.msg
   ObstacleInfo.msg
   Lane.msg
   Waypoint.msg
   Coordinates.msg
   Path.msg
   StringArray.msg
   ImageObj.msg
   ImageRect.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  )
