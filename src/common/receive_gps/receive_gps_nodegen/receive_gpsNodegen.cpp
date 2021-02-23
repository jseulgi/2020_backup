#include "src/receive_gps.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Receive_GPS_Nodegen");
	ROS_INFO("Start receiving gps...\n");
	
	GPS G;
	ros::spin();
	return 0;
}
