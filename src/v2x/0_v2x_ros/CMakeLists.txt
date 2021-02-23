cmake_minimum_required(VERSION 2.8.3)
project(v2x_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  path_msgs
  obj_msgs
  novatel_gps_msgs
  mission_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    PVD.msg
#    BSM.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v2x_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/v2x_ros
  include/j2735
  include/skeletons
  ${catkin_INCLUDE_DIRS}
  ${path_msgs_INCLUDE_DIRS}
)

## Declare a C++ library
find_library(
  asncodec_LIBRARIES
  NAMES libasncodec.a
  HINTS include/j2735
  REQUIRED)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/v2x_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_v2x_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#//////////////////// MISSION ////////////////////
#//////////////////// MISSION ////////////////////
#//////////////////// MISSION ////////////////////

# add_executable(v2x_client_node src/client_position.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_node ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v1 src/client_v1.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v1 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v2 src/client_v2.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v2 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v3 src/client_v3.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v3 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
#
# add_executable(v2x_client_v4 src/client_v4.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v4 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
#
# add_executable(v2x_client_v5 src/client_v5.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v5 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
#
# add_executable(v2x_client_v5_timer src/client_v5_timer.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v5_timer ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
#

# add_executable(v2x_client_v6 src/mission/client_v6.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v6 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v6_main src/mission/client_v6_main.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v6_main ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v6_thread src/mission/client_v6_thread.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v6_thread ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v7 src/mission/client_v7.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v7 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v70 src/mission/client_v70.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v70 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v8 src/mission/client_v8.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v8 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v9 src/mission/client_v9.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v9 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_v10 src/mission/client_v10.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(v2x_client_v10 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(test_subscribing src/test_subscribing.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(test_subscribing ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
#
# add_executable(payload_pub src/payload_pub.cpp include/v2x_ros/libcrc16.cpp)
# target_link_libraries(payload_pub ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

#//////////////////// TRAFFIC ////////////////////
#//////////////////// TRAFFIC ////////////////////
#//////////////////// TRAFFIC ////////////////////

# add_executable(v2x_client_node src/client.cpp)
# target_link_libraries(v2x_client_node ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_server_node src/server.cpp)
# target_link_libraries(v2x_server_node ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(client_v00 src/traffic/client_v00.cpp)
# target_link_libraries(client_v00 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(client_v01 src/traffic/client_v01.cpp)
# target_link_libraries(client_v01 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(client_v02 src/traffic/client_v02.cpp)
# target_link_libraries(client_v02 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(client_v03 src/traffic/client_v03.cpp)
# target_link_libraries(client_v03 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(client_v04 src/traffic/client_v04.cpp)
# target_link_libraries(client_v04 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# add_executable(v2x_client_simulated src/client_sub.cpp)
# target_link_libraries(v2x_client_simulated ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

# TO CREATE AN OBJECT LIST
# add_executable(object_list src/traffic/object_list.cpp)
# target_link_libraries(object_list ${catkin_LIBRARIES})


# -------------------------------------- FINAL VERSIONS -------------------------------------- #
add_executable(traffic_v05 src/traffic/traffic_v05.cpp)
target_link_libraries(traffic_v05 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})

add_executable(mission_v11 src/mission/mission_v11.cpp include/v2x_ros/libcrc16.cpp)
target_link_libraries(mission_v11 ${catkin_LIBRARIES} ${asncodec_LIBRARIES})
