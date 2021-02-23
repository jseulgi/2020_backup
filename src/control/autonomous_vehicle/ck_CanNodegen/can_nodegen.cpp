#include "ros/ros.h"
#include "src/CanClass.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_nodegen");
    CAN Can_class;
    ros::spin();
    return 0;
}
