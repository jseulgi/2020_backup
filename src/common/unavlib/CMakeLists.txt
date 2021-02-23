cmake_minimum_required(VERSION 2.8.3)
project(unavlib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  roslaunch
  cv_bridge
  message_generation
  pcl_conversions
  pcl_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
)

#find_package(octomap REQUIRED)
#include_directories(${OCTOMAP_INCLUDE_DIRS})
#target_link_libraries(${OCTOMAP_LIBRARIES})

#find_package(CUDA QUIET REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
#find_package(VTK REQUIRED)

set(SUBSYS_NAME unavlib)
set(SUBSYS_DESC "url navigation library")
set(SUBSYS_DEPS common)
set(LIB_NAME "unavlib_essential")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS ${SUBSYS_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${LIB_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")


set(lib_source
    src/unavlib/convt.cpp
    src/unavlib/others.cpp
)

set(unavlib_headers
    include/unavlib/utm.h
    include/unavlib/convt.h
    include/unavlib/others.h
)


add_library("${LIB_NAME}" ${unavlib_headers} ${lib_source})
target_link_libraries("${LIB_NAME}" ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRRIES})




#install(TARGETS unavlib
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#

#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE)

