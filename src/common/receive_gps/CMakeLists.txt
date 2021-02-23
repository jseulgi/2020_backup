cmake_minimum_required(VERSION 2.8.3)
project(receive_gps)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED
	message_generation
	roscpp
	std_msgs
	tf
	geometry_msgs
	path_msgs
	novatel_gps_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 src
)

set(receive_gps_source
	receive_gps_nodegen/src/receive_gps.cpp
	receive_gps_nodegen/src/receive_gps.h
)

add_executable(receive_gps receive_gps_nodegen/receive_gpsNodegen.cpp ${receive_gps_source})
add_dependencies(receive_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(receive_gps ${catkin_LIBRARIES})
