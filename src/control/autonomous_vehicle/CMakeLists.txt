cmake_minimum_required(VERSION 2.8.3)
project(autonomous_vehicle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  control_msgs
  tf
  geometry_msgs
)


add_message_files(
    FILES
    CanFrame.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ControlStatus.srv
 )



## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

set(CanNodegen_source
    ck_CanNodegen/src/CanNodegen.cpp
    ck_CanNodegen/src/CanClass.h
    )

 ##Service
 add_executable(CanNodegen ck_CanNodegen/can_nodegen.cpp ${CanNodegen_source})
 target_link_libraries(CanNodegen ${catkin_LIBRARIES})
 add_dependencies(CanNodegen CanNodegen_generate_messages_cpp)

# ##Client
# add_executable(service_client src/can_client.cpp)
# add_dependencies(service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(service_client ${catkin_LIBRARIES})
