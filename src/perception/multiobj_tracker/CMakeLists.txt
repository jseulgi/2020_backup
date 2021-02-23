cmake_minimum_required(VERSION 2.8.3)
project(multiobj_tracker)
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    tf
    tf2
    tf2_eigen
    tf2_ros
    roscpp
    std_msgs
    sensor_msgs
    novatel_gps_msgs
    nav_msgs
    obj_msgs
    control_msgs
    message_generation
    cv_bridge
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GDAL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

## Grid ROI Generator
set(obj_gridroi_source
    obj_gridROI_generator/src/gridROIgenerator.h
    obj_gridROI_generator/src/gridROIgenerator.cpp
)
add_executable(gridROI_generator obj_gridROI_generator/grid_roi_generation.cpp ${obj_gridroi_source})
add_dependencies(gridROI_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gridROI_generator ${catkin_LIBRARIES} ${GDAL_LIBRARIES})

## Object Detection by LiDAR
set(obj_detector_source
    obj_detector_lidar/src/objdetector.cpp
    obj_detector_lidar/src/objdetector.h
)
add_executable(lidar_detector obj_detector_lidar/obj_detector.cpp ${obj_detector_source})
add_dependencies(lidar_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_detector ${catkin_LIBRARIES} )

## Object Tracker (Sort)

## Object Classifier with LiDAR
set(obj_classifier_source
    obj_classifier_lidar/src/objclassifier.cpp
    obj_classifier_lidar/src/objclassifier.h
)
add_executable(lidar_classifier obj_classifier_lidar/obj_classifier.cpp ${obj_classifier_source})
add_dependencies(lidar_classifier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_classifier ${catkin_LIBRARIES})
