#include "src/path_following.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "Path_Following");
	PurePursuit  purepursuit;
	ros::spin();
	return 0;
}
