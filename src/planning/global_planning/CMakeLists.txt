cmake_minimum_required(VERSION 2.8.3)
project(global_planning)

set(CMAKE_BUILD_TYPE "release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED
	message_generation
	roscpp
	control_msgs
	path_msgs
	std_msgs
	tf
	geometry_msgs
	obj_msgs
	mission_msgs
)

find_package(GDAL REQUIRED)

 add_service_files(
   FILES
   hd_map_service.srv
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
 ${control_msgs_INCLUDE_DIRS}
 ${path_msgs_INCLUDE_DIRS}
 ${obj_msgs_INCLUDE_DIRS}
 ${mission_msgs_INCLUDE_DIRS}
)

include_directories( include ${GDAL_INCLUDE_DIRS})

set(global_planning_source
	global_planning_nodegen/src/global_planning.cpp
	global_planning_nodegen/src/global_planning.h
)

add_executable(global_planning global_planning_nodegen/global_planningNodegen.cpp ${global_planning_source})
add_dependencies(global_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_planning ${catkin_LIBRARIES})
target_link_libraries(global_planning ${GDAL_LIBRARIES})
