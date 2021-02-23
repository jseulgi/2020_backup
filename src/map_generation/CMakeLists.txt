cmake_minimum_required(VERSION 2.8.3)
project(map_generation)

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
)

include_directories( include ${GDAL_INCLUDE_DIRS})

add_executable(map_generation src/map_generation_node.cpp src/map_generation.cpp)
add_dependencies(map_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_generation ${catkin_LIBRARIES})
target_link_libraries(map_generation ${GDAL_LIBRARIES})
