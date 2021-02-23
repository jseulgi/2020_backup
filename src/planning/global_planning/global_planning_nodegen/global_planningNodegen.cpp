#include "src/global_planning.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Global_Planning_Nodegen");
	ROS_INFO("Start operating global path planning...\n");
	PLANNING PlanningClass;
	ros::spin();
	return 0;
}
