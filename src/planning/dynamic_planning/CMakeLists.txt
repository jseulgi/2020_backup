cmake_minimum_required(VERSION 2.8.3)
project(dynamic_planning)
set(CMAKE_BUILD_TYPE Release)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	path_msgs
	obj_msgs
	multiobj_tracker
	control_msgs
	tf
	geometry_msgs
	novatel_gps_driver
	tf_conversions
)

find_package(GDAL REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 src
 ${path_msgs_INCLUDE_DIRS}
 ${obj_msgs_INCLUDE_DIRS}
 ${control_msgs_INCLUDE_DIRS}
 ${multiobj_tracker_INCLUDE_DIRS}
)

add_executable(dynamic_planning_node src/dynamic_planning.cpp src/dynamic_planning_node.cpp)
add_dependencies(dynamic_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_planning_node ${catkin_LIBRARIES})
