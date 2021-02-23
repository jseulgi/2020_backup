/**
  @ class 	MapGen
  @ date	2020/01
  @ author 	Seulgi Jeon, CKS
  @ description	: Generate node which contains global path data
*/

#ifndef MAP_GEN_H
#define MAP_GEN_H

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
#include "ogrsf_frmts.h"
#include <time.h>
#include <ctime>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define math_pi 3.1415926535897931

using namespace std;
using namespace path_msgs;

class MapGen
{
public:
	MapGen();
	~MapGen();

private:
	ros::NodeHandle nh;/**< ros node handler */
	ros::Publisher MapLineStripPub;/**< map line strip publisher for rivz*/
	ros::Publisher MapPointsPub;
	ros::Publisher LinkArrayPub;
	ros::Publisher LaneArrayPub;
	ros::Publisher StopArrayPub;
	ros::Publisher PathArrayPub;
	ros::Publisher OffsetMapPub;

	// Euigon
	ros::Publisher GlobalMapPub;
	Map GlobalMap;

	double OffsetMapX;
	double OffsetMapY;
	geometry_msgs::Point OffsetMap;

	geometry_msgs::Point StopPoint;
	visualization_msgs::MarkerArray LaneArray;
	visualization_msgs::MarkerArray LinkArray;
	visualization_msgs::MarkerArray StopArray;
	visualization_msgs::Marker MapLineStrip;
	visualization_msgs::Marker MapPoints;
	visualization_msgs::Marker PathArray;

	void SetParams();
	void ReadShapeFile(string file_name, string layer_name);
	void InitializeRvizPoints();
	void InitializeRvizLaneArray();
	void InitializeRvizLinkArray();
	vector<geometry_msgs::Point> ParsingLaneGeometry(string geometry_str);
	vector<geometry_msgs::Point> ParsingLinkGeometry(string geometry_str);
	string ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField);
	void OpenNodeSHP();
	void OpenLinkSHP();
	void OpenLaneSHP();

	/**< Parameters of NodeGen set from the rosparam */
	bool is_link_on;
	bool is_lane_on;
	bool is_node_on;
	bool is_path_on;

	bool is_simul_on;

	string abspath;
	string lane_shp;
	string link_shp;
	string node_shp;
	// string stop_shp;

	string start_node_id;
	string goal_node_id;

	string layer_lane;
	string layer_link;
	string layer_node;

	int shp_lane_id;
	int shp_lane_right_link;
	int shp_lane_left_link;
	int shp_lane_lanetype;
	int shp_lane_lanecode;
	int shp_lane_barrier;
	int shp_lane_laneno;
	int shp_lane_code;

	int shp_link_id;
	int shp_link_from_node;
	int shp_link_to_node;
	int shp_link_road_type;
 	int shp_link_length;
	int shp_link_speed;
	int shp_link_left_link;
	int shp_link_right_link;
	int shp_link_next_link;

	int shp_node_id;
	int shp_node_next_link;
	int shp_node_prev_link;

	map<string, Node> nodes;
	map<string, Link> links;
	map<string, Lane> lanes;

};

#endif
