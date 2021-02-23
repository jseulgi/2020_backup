cmake_minimum_required(VERSION 2.8.3)
project(path_following)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  control_msgs
  roscpp
  std_msgs
  nav_msgs
  path_msgs
  tf
  tf2
  geometry_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
  ${control_msgs_INCLUDE_DIRS}
  ${path_msgs_INCLUDE_DIRS} 
)

set(path_following_source
	path_following_nodegen/src/path_following.cpp
	path_following_nodegen/src/path_following.h)

add_executable(path_following path_following_nodegen/path_followingNodegen.cpp ${path_following_source})
add_dependencies(path_following ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_following ${catkin_LIBRARIES})
