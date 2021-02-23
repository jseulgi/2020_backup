#include "map_generation/map_generation.h"


int main(int argc, char **argv)
{

	ros::init(argc, argv, "map_generation_node");
	MapGen node;
	ros::spin();
	return 0;
}
