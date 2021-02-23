cmake_minimum_required(VERSION 2.8.3)
project(jslocalization)
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    sensor_msgs
    roslaunch
    std_msgs
    unavlib
    cv_bridge
    laser_geometry
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf
    tf2
    tf2_eigen
    tf2_ros
    novatel_gps_msgs
    nav_msgs
    localization_msgs
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs

   )

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GDAL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs localization_msgs
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${GDAL_INCLUDE_DIRS}
 ${CERES_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 ${Boost_INCLUDE_DIRS}
)

## NODE & ODOM GENERATION ##
set(nodegen_source
    nodegen/src/nodegen.cpp
    nodegen/src/nodegen.h
)

add_executable(nodegen nodegen/main.cpp ${nodegen_source})
add_dependencies(nodegen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodegen ${catkin_LIBRARIES} ${GDAL_LIBRARIES} ${PCL_LIBRARIES})

## EXTRACT FEATURE ##
#set(extractfeature_source
#    extractfeature/src/extractfeature.cpp
#    extractfeature/src/extractfeature.h
#)

#add_executable(extractfeature extractfeature/main.cpp ${extractfeature_source})
#add_dependencies(extractfeature ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(extractfeature ${catkin_LIBRARIES} ${GDAL_LIBRARIES} ${PCL_LIBRARIES})

## MONTE CARLO LOCALIZATION ##
#set(mcl_source
#    mcl/src/mcl.cpp
#    mcl/src/mcl.h
#)
#add_executable(mcl mcl/main.cpp ${mcl_source})
#add_dependencies(mcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mcl ${catkin_LIBRARIES} ${GDAL_LIBRARIES} ${PCL_LIBRARIES})

