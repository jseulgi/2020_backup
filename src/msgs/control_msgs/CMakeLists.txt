cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs    
    geometry_msgs
    nav_msgs
    message_generation
    )

add_message_files(
    FILES
    CanFrame.msg
    Vehicle_cmd.msg
    VehicleState.msg
)

add_service_files(
  FILES

)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    )


catkin_package(CATKIN_DEPENDS message_runtime)
