/**
  @ class 	PLANNING
  @ date	2020/01
  @ author 	Seulgi Jeon, CKS
  @ description	: Generate node which contains global path data
*/

#ifndef GLOBAL_PLANNING_H
#define GLOBAL_PLANNING_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
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
#include "path_msgs/Node.h"
#include "path_msgs/Link.h"
#include "path_msgs/Cost.h"
#include "path_msgs/Bezier.h"
#include "path_msgs/Trajectory.h"
#include "path_msgs/Waypoint.h"
#include "path_msgs/Map.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "conversions.h"
#include "nav_msgs/Odometry.h"
#include "ogrsf_frmts.h"
#include <time.h>
#include <ctime>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
#include "mission_msgs/mission_list.h"
#include "mission_msgs/mission.h"
#include "mission_msgs/global2mission.h"
#include "mission_msgs/global2mission_list.h"
#include <algorithm>

#define math_pi 3.1415926535897931

using namespace std;
using namespace gps_common;
using namespace path_msgs;

class PLANNING
{
public:
	PLANNING();
	~PLANNING();

private:
	ros::NodeHandle nh;/**< ros node handler */
	ros::Publisher OdometryPub;/**< odometry publisher */
	ros::Publisher MapLineStripPub;/**< map line strip publisher for rivz*/
	ros::Publisher MapPointsPub;
	ros::Publisher LinkArrayPub;
	ros::Publisher LaneArrayPub;
	ros::Publisher StopArrayPub;
	ros::Publisher PathArrayPub;
	ros::Publisher GlobalPathPub;
	ros::Publisher OffsetMapPub;
	ros::Publisher Global2MissionPub;
	ros::Publisher Global2MissionStaticPub;
	ros::Publisher Start_debugPub;
	ros::Publisher End_debugPub;
	ros::Publisher debug1Pub;
	ros::Publisher debug2Pub;
	ros::Publisher debug3Pub;
	ros::Publisher PathArray2startPub;
	ros::Publisher StaticObjLinkIdPub;
	ros::Publisher GoalTestPub;
	// Euigon
	ros::Publisher TrajectoryPub;
	ros::Publisher GlobalMapPub;
	ros::Publisher rviz_current_pose_pub_;
	tf::TransformBroadcaster tfBroadcaster;

	// tf::StampedTransform MapTrans;
	// tf::TransformBroadcaster tfBroadcaster;


	double rot_cov;
	double OffsetMapX;
	double OffsetMapY;
	int bezier_size;
	geometry_msgs::Point OffsetMap;

	geometry_msgs::Point StopPoint;
	visualization_msgs::MarkerArray LaneArray;
	visualization_msgs::MarkerArray LinkArray;
	visualization_msgs::MarkerArray StopArray;
	visualization_msgs::Marker MapLineStrip;
	visualization_msgs::Marker MapPoints;
	visualization_msgs::Marker PathArray;
	visualization_msgs::Marker PathArray2start;


	void SetParams();
	void ReadShapeFile(string file_name, string layer_name);
	void InitializeRvizPoints();
	void InitializeRvizLaneArray();
	void InitializeRvizLinkArray();
  void GlobalPathHandler(const geometry_msgs::PoseStampedPtr& NavGoalMsg);
  void CopyGoalPointfromRviz(const geometry_msgs::PoseStampedPtr& NavGoalMsg);
	void GenerateGlobalPath();
	void UpdateOpenList(Cost temp_cost);
	void DrawingPath();
	void DrawingPath_debug1(Trajectory traj);
	void DrawingPath_debug2(Trajectory traj);
	void GenerateWayPoint(vector<Cost> closed_list);
	void correctWaypoints(Trajectory &traj);
	vector<geometry_msgs::Point> GenerateCurvePoint(Link prev_link, Link waypoint_link);
	Cost InitializeCost();
	Cost CalcCost(Node current_node, Link current_link, Node goal_node);
	Cost CalcCost(Node current_node, Link current_link, Link lrlink, Node goal_node);
	float GetDistance(float pCurrent_x, float pCurrent_y, float pNext_x, float pNext_y);

	vector<geometry_msgs::Point> ChangeLane(Link from_link, Link to_link);
  Trajectory ChangeLaneTrajectory(Link from_link, Link to_link);
	vector<geometry_msgs::Point> GoalLinkChangeLane(Link from_link, Link to_link);
  Trajectory GoalLinkChangeLaneTrajectory(Link from_link, Link to_link);

	double CalcLength(Link link, int from_idx, int to_idx);
	int SearchIndex(Link link, int from_idx, int to_idx, double refer_length);
	int SearchClosestIndex(geometry_msgs::Point point, Link link);

	// CK //
	void RvizNavStartCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& NavStartMsg);
	void RvizNavGoalCallback(const geometry_msgs::PoseStamped &NavGoal);
	void mapInfoCallback(const path_msgs::Map map);
	void currentOdomCallback(const nav_msgs::Odometry msg);
	void visualizeCurrentPose();
	// void BaseLinkOffsetCallback(const geometry_msgs::Point);
	// void GpsCallback(const sensor_msgs::NavSatFixConstPtr& fix);
	// void GpsHeadingCallback(const ublox_msgs::NavPVTConstPtr& navheading);
	double GetSlope(geometry_msgs::Point a, geometry_msgs::Point b);
	double GetIntersectY(double slope, geometry_msgs::Point a);
	double RvizPose2yaw(geometry_msgs::Pose pose);

	void GenerateGoalPoint();
	void GenerateStartPoint(bool is_lane_changed);
	double GetSign(double angle);

	pair<string, geometry_msgs::Point> FindClosestLink(geometry_msgs::Pose CurPose);
	pair<string, geometry_msgs::Point> ClosestStartLink, ClosestEndLink;
	pair<string, geometry_msgs::Point> ClosestEndPrevLink;

    /*---------------------------------------------------**/

	map<string, Node> nodes;
	map<string, Link> links;
	map<string, Lane> lanes;

	vector<string> trajectory_link;
	vector<string> trajectory_node;
	vector<Cost> open_list;
	vector<Cost> closed_list;

	bool is_simul_on;
	bool map_initialized_;
	bool pose_initialized_;
	bool is_lane_changed;
	double length_lane_changed;
	double weight;

	string start_node_id;
	string goal_node_id;

	vector<geometry_msgs::Point> trajectory_points;

  // Euigon
  Trajectory trajectory;
  std::vector<Trajectory> vector_trajectory;
  std::vector<Trajectory> vector_trajectory2start;

  // CK //

	vector<pair<string, geometry_msgs::Point>> GeometryPair;

	double NavheadingValueRad;
	double NavheadingValueDeg;
	double StartLinkSlope;
	double EndLinkSlope;
	string VehicleClosestLinkId;
	geometry_msgs::Point VehicleClosestGeo;
	geometry_msgs::Pose RvizPosition;
	geometry_msgs::Pose RvizGoal;
	geometry_msgs::Point OffsetGps;
	geometry_msgs::Pose VehiclePosition;
    geometry_msgs::PoseStamped CurrentPose;

	double push_length;
	double bezier_push_length;

	double weight_lane;
	double atypical_weight;

	void globalObjCallback(const obj_msgs::ObjList msg);
	void missionCallback(const mission_msgs::mission_list mission_msg);
	void CopyGoalPointfromRviz2(const geometry_msgs::Pose NavGoalMsg);
	// void selectedmissionCallback(const)
	std::vector<obj_msgs::Obj> vector_globalObj;
	std::vector<geometry_msgs::Pose> vector_globalObj_pose;
	std::vector<pair<string, geometry_msgs::Point>> vector_globalObj_link;
	std::vector<std::string> vector_globalObj_link_str;
	std::vector<mission_msgs::mission> vector_mission;
	std::vector<geometry_msgs::Pose> vector_start;
	std::vector<geometry_msgs::Pose> vector_end;
	bool mission_flag;
	bool sent_flag;
	std::vector<pair<int,int>> vector_calculated_missions; // ID, # of Points
	std::vector<pair<int,int>> vector_calculated2start_pos; // ID, # of Points
	std::vector<pair<int,int>> vector_calculated_missions_final; //Id , # of Points
	std::vector<geometry_msgs::Point> num_point_mission;
	std::vector<pair<string, geometry_msgs::Point>> vector_closest_start; // Closest link and point from start_pos
	std::vector<pair<string, geometry_msgs::Point>> vector_closest_end; // Closest link and point from end_pos
	std::vector<pair<string, geometry_msgs::Point>> vector_closest_start_vehicle; // Closest link and point from vehicle
	std::vector<pair<int,int>> vector_calculated_missions2start;

	mission_msgs::global2mission global2mission_msg;
	mission_msgs::global2mission_list global2mission_msg_list;
	mission_msgs::global2mission global2mission_static_msg;

	int start_goal_flag;
	// 0: Start exists before End on same link
	// 1: Start exists after End on same link
	// 2: Start and End exist at different link
	pair<string, geometry_msgs::Point> first_closest_link;

	int debug_num;

	obj_msgs::ObjList StaticObj_list;
	obj_msgs::Obj StaticObj;
};

#endif
