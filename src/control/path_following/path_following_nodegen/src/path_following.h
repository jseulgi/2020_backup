#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include "ros/ros.h"
#include "ros/timer.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <cstring>
#include <vector>
#include <cmath>
#include <deque>

#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "control_msgs/CanFrame.h"
#include "control_msgs/Vehicle_cmd.h"
#include "control_msgs/VehicleState.h"
#include "path_msgs/State.h"
#include "novatel_gps_msgs/Inspva.h"

#include "nav_msgs/Odometry.h"

//Convert GPS/fix to Odom (Utm52N)
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <gps_common/GPSFix.h>
#include <novatel_gps_msgs/NovatelHeading2.h>

using namespace std;

class PurePursuit
{
public:
	PurePursuit();
	~PurePursuit();

	// void NavheadingCallback(const ublox_msgs::NavPVTConstPtr& navheading);
	// void GpsCallback(const sensor_msgs::NavSatFixConstPtr& fix);
	void GpsCallback(const novatel_gps_msgs::InspvaConstPtr& fix);
	void GlobalPathCallback(const visualization_msgs::MarkerConstPtr& globalpath);
	void roadStateCallback(const path_msgs::State msg);
	void callback_vehicle_state(const control_msgs::VehicleState &msg);
	void refPathCallback(const visualization_msgs::Marker path);
	void currentOdomCallback(const nav_msgs::OdometryPtr& msg);
	float Point2PointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	double GetSign(double angle);
	void FindClosestGeometryIndex(vector<geometry_msgs::Point> path);
	void SteeringControl();
	void MarkerVisualization();

private:
	ros::NodeHandle nh;

	ros::Publisher OdomPub;
	ros::Publisher LookAheadMarkerPub;
	ros::Publisher VehicleTargetSteeringAngle_pub;
	ros::Publisher lookahead_dist_pub_;
	// Subscribed Variables
	vector<geometry_msgs::Point> ref_path_;
	string frame_id, child_frame_id;
	geometry_msgs::Point hood_pos_;
	double rot_cov;

	int road_state_;
	std::deque<float> window;
	nav_msgs::Odometry VehicleGpsOdom;
	geometry_msgs::Point VehicleGpsPosition;
	geometry_msgs::Pose current_pose_;
	visualization_msgs::Marker LookAheadMarker;
	float LookAheadPointTheta;
	float SteeringCommandAngle;
	float LookAheadDistance;
	control_msgs::VehicleState VehicleState;
	control_msgs::Vehicle_cmd Vehicle_cmd;

};

#endif // PATH_TRACKING_H
