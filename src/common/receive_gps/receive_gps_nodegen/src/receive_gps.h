#ifndef RECEIVE_GPS_H
#define RECEIVE_GPS_H

#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "conversions.h"
#include "nav_msgs/Odometry.h"
#include <time.h>
#include <ctime>
#include <cstdio>
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "novatel_gps_msgs/Inspva.h"
#include "path_msgs/Map.h"


#define M_PI 3.1415926535897931

using namespace std;
using namespace gps_common;

class GPS
{
public:
	GPS();
	~GPS();

private:
	ros::NodeHandle nh;/**< ros node handler */

	ros::Publisher pubGpsOdom;
	ros::Subscriber subGpsIns;
	ros::Subscriber subMapInfo;
	tf::TransformBroadcaster tfBroadcaster;

	double offset_x;
	double offset_y;
  float NavheadingRad;
  bool map_initialized_;

  void Initialization();
  void NovatelCallback(const novatel_gps_msgs::InspvaPtr& gpsHandler);
  void mapInfoCallback(const path_msgs::Map msg);

};

#endif
