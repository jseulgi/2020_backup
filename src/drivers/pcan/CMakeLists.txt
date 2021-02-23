cmake_minimum_required(VERSION 2.8.3)
project(pcan)

# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
)

find_package(pcanbasic)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(pcan_pub_write src/pcan_pub_write.cpp)
#add_executable(pcan_read src/pcan_read.cpp)

# The C++ compiler of Ubuntu 14.04 is upgraded. We have to put '-lstdc++' in here for using standard c++ compiler.
#-lstdc++

target_link_libraries(pcan_pub_write
  ${catkin_LIBRARIES}
  pcanbasic
)

#target_link_libraries(pcan_read
#  ${catkin_LIBRARIES}
#  pcanbasic
#)

