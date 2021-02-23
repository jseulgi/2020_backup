#include "src/mcl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monte carlo localization");

   	mcl PF;

    ros::spin();

    return 0;
}
