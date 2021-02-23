#include "path_following.h"
#define math_pi 3.1415926535897931

PurePursuit::PurePursuit(){

	LookAheadDistance = 8;
	static ros::Subscriber VelocitySub = nh.subscribe("/vehicle_state",1,&PurePursuit::callback_vehicle_state,this);
	static ros::Subscriber RefPathSub = nh.subscribe("/ref_path", 1, &PurePursuit::refPathCallback, this);
	static ros::Subscriber current_odom_sub_ = nh.subscribe("/gps_odom", 1, &PurePursuit::currentOdomCallback, this);
	static ros::Subscriber road_state_sub_ = nh.subscribe("/road_state", 1, &PurePursuit::roadStateCallback, this);
	static ros::Rate *rate = new ros::Rate(100);

	OdomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom",1);
	LookAheadMarkerPub = nh.advertise<visualization_msgs::Marker>("look_ahead_point_marker",1);
	lookahead_dist_pub_ = nh.advertise<std_msgs::Float64>("/lookahead_distance", 1);
	VehicleTargetSteeringAngle_pub = nh.advertise<control_msgs::Vehicle_cmd>("/Steering_cmd",1);

	while (ros::ok())
  {
    ros::spinOnce();

		vector<geometry_msgs::Point> current_path = ref_path_;

		if (current_path.size() > 0)
		{
			FindClosestGeometryIndex(current_path);
			MarkerVisualization();
			SteeringControl();
			printf("LookAhead: %f m\n", LookAheadDistance);
			std_msgs::Float64 lah;
			lah.data = LookAheadDistance;
			lookahead_dist_pub_.publish(lah);
		}
		rate->sleep();
  }
}

PurePursuit::~PurePursuit(){

}
void PurePursuit::roadStateCallback(const path_msgs::State msg)
{
	road_state_ = msg.state;
}

void PurePursuit::callback_vehicle_state(const control_msgs::VehicleState &msg)
{
	double k = 0.4;

	VehicleState.v_ego = msg.v_ego;

	// ATypical turn && Before Turn
	if (road_state_ == 8 || road_state_ == 10)
	{
		LookAheadDistance = 6;
	}
	// // ATypical straight
	// else if (road_state_ == 6)
	// {
	// 	LookAheadDistance = 8;
	// }
	else
	{
		if(VehicleState.v_ego < 20)
			LookAheadDistance = 8;
		else if(VehicleState.v_ego < 40)
			LookAheadDistance = k * VehicleState.v_ego;
		else
			LookAheadDistance = 16;
	}
}


void PurePursuit::currentOdomCallback(const nav_msgs::OdometryPtr& msg)
{
	current_pose_ = msg->pose.pose;
	double yaw = tf::getYaw(current_pose_.orientation);
	hood_pos_.x = current_pose_.position.x + 2.5*cos(yaw);
	hood_pos_.y = current_pose_.position.y + 2.5*sin(yaw);
}

void PurePursuit::refPathCallback(const visualization_msgs::Marker path)
{
	ref_path_ = path.points;
}
//
// void PurePursuit::GlobalPathCallback(const visualization_msgs::MarkerConstPtr& path){
// 	GlobalPath.clear();
// 	for(auto i=0; i<path->points.size(); i++){
// 		GlobalPath.push_back(path->points[i]);
// 	}
// 	cout << "Received GlobalPath Points: " << GlobalPath.size() << endl;
// 	is_path_generated = true;
// }

// void PurePursuit::NavheadingCallback(const ublox_msgs::NavPVTConstPtr& navheading){
// 	double NavheadingDeg = navheading->heading * 0.00001;
// 	NavheadingDeg = - (NavheadingDeg - 90); // Yaw in Degree
// 	NavheadingRad = (NavheadingDeg * math_pi) / 180; // Yaw in Radiance (Global Variable)

// 	// cout << "Yaw in Rad: " << NavheadingRad << endl;
// }

void PurePursuit::MarkerVisualization(){
	LookAheadMarker.header.frame_id = "map";
	LookAheadMarker.header.stamp = ros::Time::now();
	LookAheadMarker.action = visualization_msgs::Marker::ADD;
	LookAheadMarker.pose.orientation.w = 1.0;
	LookAheadMarker.id = 0;
	LookAheadMarker.type = visualization_msgs::Marker::POINTS;
	LookAheadMarker.scale.x = 2;
	LookAheadMarker.scale.y = 2;
	LookAheadMarker.color.r = 1.0;
	LookAheadMarker.color.a = 1.0;

	LookAheadMarkerPub.publish(LookAheadMarker);

	LookAheadMarker.points.clear();

}

float PurePursuit::Point2PointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
	float distance = sqrt( pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2));
	return distance;
}

double PurePursuit::GetSign(double angle){
	if(angle >= 0.0){
		return 1.0;
	}
	else{
		return -1.0;
	}
}

void PurePursuit::FindClosestGeometryIndex(vector<geometry_msgs::Point> path){
	int index = 0;
	float min = 9999;
	float dist;
	float sum = 0.0;
	float difftheta;
	float heading;
	float path_heading;
	double sign;

	for (int i=1; i<path.size(); i++){
		sum += Point2PointDistance(path[i-1], path[i]);
		heading = tf::getYaw(current_pose_.orientation);
		// path_heading = atan2(path[i].y - path[0].y, path[i].x - path[0].x);
		path_heading = atan2(path[i].y - current_pose_.position.y, path[i].x - current_pose_.position.x);
		difftheta = heading - path_heading;
		//
		// cout << "heading: " << heading << endl;
		// cout << "delta y: " << path[i].y - current_pose_.position.y << endl;
		// cout << "delta x: " << path[i].x - current_pose_.position.x << endl;
		// cout << "tan: " << atan2(path[i].y - current_pose_.position.y, path[i].x - current_pose_.position.x) << endl;
		// cout << "diff: " << difftheta << endl;

		// if (abs(difftheta) > math_pi/2)
		// {
		// 	continue;
		// }

		if(abs(difftheta) >= math_pi){
			sign = GetSign(difftheta);
			difftheta = difftheta - (sign * 2 * math_pi);
		}

		if(sum > LookAheadDistance){
			index = i;
			LookAheadPointTheta = difftheta;
			// printf("Actual distance: %f\n", sum);
			break;
		}
	}

	// cout << "Size: " << path.size() << endl;
	cout << " Index: " << index << endl;
	if (index == 0)
	{
		printf("=============INDEX 0000000000=========\n");
		geometry_msgs::Point last_point = path[path.size()-1];
		// path_heading = atan2(last_point.y-path[0].y, last_point.x - path[0].x);
		path_heading = atan2(last_point.y-current_pose_.position.y, last_point.x - current_pose_.position.x);

		difftheta = heading - path_heading;
		if(abs(difftheta) >= math_pi){
			sign = GetSign(difftheta);
			difftheta = difftheta - (sign * 2 * math_pi);
		}
		index = path.size()-1;
		LookAheadPointTheta = difftheta;
	}
	// cout << "Diff_theta: " << LookAheadPointTheta << endl;

	LookAheadMarker.points.push_back(path[index]);
}
//
// void PurePursuit::FindClosestGeometryIndex(vector<geometry_msgs::Point> path){
// 	int index = 0;
// 	float min = 9999;
// 	float dist;
// 	float difftheta;
// 	float heading;
// 	float path_heading;
// 	double sign;
//
// 	for (int i=0; i<path.size(); i++){
// 		dist = Point2PointDistance(path[i], current_pose_.position);
// 		heading = tf::getYaw(current_pose_.orientation);
// 		path_heading = atan2(path[i].y - current_pose_.position.y, path[i].x - current_pose_.position.x);
// 		difftheta = heading - path_heading;
// 		//
// 		// cout << "heading: " << heading << endl;
// 		// cout << "delta y: " << path[i].y - current_pose_.position.y << endl;
// 		// cout << "delta x: " << path[i].x - current_pose_.position.x << endl;
// 		// cout << "tan: " << atan2(path[i].y - current_pose_.position.y, path[i].x - current_pose_.position.x) << endl;
// 		// cout << "diff: " << difftheta << endl;
//
// 		// if (abs(difftheta) > math_pi/2)
// 		// {
// 		// 	continue;
// 		// }
//
// 		if(abs(difftheta) >= math_pi){
// 			sign = GetSign(difftheta);
// 			difftheta = difftheta - (sign * 2 * math_pi);
// 		}
//
// 		if(dist > LookAheadDistance){
// 			if(dist < min){
// 				min = dist;
// 				index = i;
// 				LookAheadPointTheta = difftheta;
// 			}
// 		}
// 	}
//
// 	// cout << "Size: " << path.size() << endl;
// 	cout << " Index: " << index << endl;
// 	if (index == 0)
// 	{
// 		min = path
// 		printf("=============INDEX 0000000000=========\n");
// 	}
// 	// cout << "Diff_theta: " << LookAheadPointTheta << endl;
//
// 	LookAheadMarker.points.push_back(path[index]);
// }

void PurePursuit::SteeringControl(){

	// steering angle = arctan (2 * front-rear-wheel-length * sin (lookaheadangle) / lookahead distance from rear axle)
	SteeringCommandAngle = atan( 2.0 * 2.7 * sin(LookAheadPointTheta) / LookAheadDistance);
	// ATypical turn && Before Turn
	if (road_state_ == 8 || road_state_ == 9 || road_state_ == 10)
	{
		SteeringCommandAngle *= 1.4;
	}
	SteeringCommandAngle = (SteeringCommandAngle * 180 / math_pi) * -13.9;

	printf("Steering: %f\n", SteeringCommandAngle);
	// window.push_back(SteeringCommandAngle);
	//
	// // Weighted Average
  // size_t window_size = 20;
	//
  // if (window.size() > window_size)
  // {
  //   float avg = 0.0;
  //   window.pop_front();
	//
  //   // Weighted average
  //   for (int i = 0; i < window_size; i++)
  //   {
  //     avg += window[i] * (i+1) / (window_size*(window_size + 1) / 2);
  //   }	// window.push_back(SteeringCommandAngle);

	// 	Vehicle_cmd.target_steering = avg;
	// 	VehicleTargetSteeringAngle_pub.publish(Vehicle_cmd);
	// 	cout << "Steer Command Avg: " << avg << endl;
	// 	cout << "Steer Command: " << SteeringCommandAngle << endl;
  // }

	Vehicle_cmd.target_steering = SteeringCommandAngle;
  VehicleTargetSteeringAngle_pub.publish(Vehicle_cmd);

}
