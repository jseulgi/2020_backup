#include "src/nodegen.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_generation");

    NodeGen nodegen;

    ros::spin();

    return 0;
}
