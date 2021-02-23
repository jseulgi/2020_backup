////////////////////////////////////////
// 	VERSION INFO
//	* this version works for the Daegu road.
//		- ros::spinonce() incorporated 				[done]
//		- offset_x, offset_y to match HD MAP 		[done]
//		- ifdef to comment out debugging outputs	[done]
//
//		- make objects at specific coordinates		[done]
//  NEXT VERSION: ...
//		-
//		-
//		-
//		-
////////////////////////////////////////

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
// COMMON
#include <iostream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <sstream>
// RVIZ VISUALIZATION
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// LLtoUTM & PATH_MSGS & OBJ_MSGS
#include "conversions.h"
#include "path_msgs/Map.h"
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"


// SIGNAL
#include <signal.h>


bool is_done_ = false;

class V2X_Client {
private:
	ros::NodeHandle private_nh_;

	// PUBs AND SUBs
		//pub
	ros::Publisher markerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("markerArray", 0);
	ros::Publisher textmarkerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("textmarkerArray", 0);
	ros::Publisher obj_list_pub = private_nh_.advertise<obj_msgs::ObjList>("/traffic_info",0);
		//sub
	ros::Subscriber sub1 = private_nh_.subscribe<path_msgs::Map>("/map_info", 1, &V2X_Client::mapInfoCallback, this);

	// CLASS VARIABLES
	std::vector<double> objects_list_x;
	std::vector<double> objects_list_y;
	int number_of_objects;

	// OBJECT LISTS
	obj_msgs::ObjList obj_list;

	// For Map Translation
	double offset_x = 0;
	double offset_y = 0;

public:
	V2X_Client(void):
		private_nh_("~")
	{
		initParams();
		ROS_INFO("V2X CLIENT NODE INITIALIZED");
	}

	void initParams(void) {
		ROS_INFO("PRINT OBJECT INFORMATION");
		if(	private_nh_.getParam("/objects_qgis/x", objects_list_x)){
			for (auto i = objects_list_x.begin(); i != objects_list_x.end(); ++i){
    			std::cout << *i << ' ';
			}
		} else {
			ROS_INFO("x param not loading... shutting down");
			ros::shutdown();
		}
		if(private_nh_.getParam("/objects_qgis/y", objects_list_y)){
			for (auto i = objects_list_y.begin(); i != objects_list_y.end(); ++i){
    			std::cout << *i << ' ';
			}
		} else {
			ROS_INFO("y param not loading... shutting down");
			ros::shutdown();
		}

		if (objects_list_x.size() != objects_list_y.size()){
			ROS_INFO("ERROR: check config/object_list.yaml since number of x,y coordinates are different \n %d != %d", (int)objects_list_x.size(), (int)objects_list_y.size());
			ros::shutdown();
		} else {
			ROS_INFO("object_list_x.size() :%d ", (int)objects_list_x.size());
			ROS_INFO("object_list_y.size() :%d ", (int)objects_list_y.size());
		}

		number_of_objects = (int) objects_list_y.size();
		ROS_INFO("number_of_objects is : %d",number_of_objects);

		// private_nh_.getParam("obu_ip", obu_ip_);
		// private_nh_.getParam("obu_port", obu_port_);
		// private_nh_.getParam("buffer_size", buff_size_);
		// private_nh_.getParam("sleep_rate", rate_);
		// private_nh_.getParam("debug_on", is_debug_on_);
	}


	void initSignalHandler(void) {
		struct sigaction sa;

		memset(&sa, 0, sizeof sa);
		sa.sa_handler = signalHandler;

		if (sigaction(SIGINT, &sa, nullptr) < 0) {
			ROS_ERROR("FAILED TO INSTALL SIGNAL HANDLER");
			ros::shutdown();
		}
		else ROS_INFO("INSTALL SIGNAL HANDLER SUCCESSFULLY");
	}

	static void signalHandler(int signal) {
		ROS_INFO("%d SIGNAL RECEIVED, TERMINATE V2X CLIENT NODE", signal);
		is_done_ = true;
		ros::shutdown();
	}




	void mapInfoCallback(const path_msgs::Map msg)
	{
		offset_x = msg.OffsetMapX;
		offset_y = msg.OffsetMapY;
	}


	int create_static_objects(){
		std::cout<<"\n------------------------------------------------------------------"<<std::endl;
		ROS_INFO("offset_x is : %f",offset_x);
		ROS_INFO("offset_y is : %f",offset_y);

		float width  = 2; //meter
		float length = 2; //meter

		// CREATE AN obj_msgs/obj FOR EACH OBJECT

		for (int i=0; i<number_of_objects ;i++){
			static obj_msgs::Obj dummy_object;
			dummy_object.id = 500+i;
			dummy_object.type = (int16_t) 10; //for traffic sign
			dummy_object.pose.position.x = objects_list_x[i] - offset_x;
			dummy_object.pose.position.y = objects_list_y[i] - offset_y;
			ROS_INFO("object %d coordinate x : %f",(int)i, (double)dummy_object.pose.position.x);
			ROS_INFO("object %d coordinate y : %f",(int)i, (double)dummy_object.pose.position.y);
			dummy_object.pose.position.z = 0;
			dummy_object.pose.orientation.x = 0;
			dummy_object.pose.orientation.y = 0;
			dummy_object.pose.orientation.z = 0;
			dummy_object.pose.orientation.w = 1;
			dummy_object.ul.x = dummy_object.pose.position.x + width/2;
			dummy_object.ul.y = dummy_object.pose.position.y + length/2;
			dummy_object.ul.z = 0;
			dummy_object.ur.x = dummy_object.pose.position.x - width/2;
			dummy_object.ur.y = dummy_object.pose.position.y + length/2;
			dummy_object.ur.z = 0;
			dummy_object.bl.x = dummy_object.pose.position.x + width/2;
			dummy_object.bl.y = dummy_object.pose.position.y - length/2;
			dummy_object.bl.z = 0;
			dummy_object.br.x = dummy_object.pose.position.x - width/2;
			dummy_object.br.y = dummy_object.pose.position.y - length/2;
			dummy_object.br.z = 0;
			dummy_object.velocity.x = 0;
			dummy_object.velocity.y = 0;
			dummy_object.velocity.z = 0;

			// ADD THE OBJECT TO THE LIST
			obj_list.objlist.push_back(dummy_object);
		}
		// Finally Publish Obj_list
		obj_list_pub.publish(obj_list);
	}

	int run(){

		//// MARKERS FOR THE OBJECTS
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::MarkerArray text_marker_array;
		visualization_msgs::Marker text_marker;
		visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "object";
			// marker.id = 0;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			// marker.pose.position.x = easting - offset_x;
			// marker.pose.position.y = northing - offset_y;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 1;
			marker.lifetime = ros::Duration(1);
			marker.scale.x = 2.0;
			marker.scale.y = 2.0;
			marker.scale.z = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;

		for(int i=0; i < number_of_objects ;i++){
			// SITUATE A MARKER AT WHERE THE OBJECTS ARE
			marker.id = i;
			marker.pose.position.x = obj_list.objlist[i].pose.position.x;
			marker.pose.position.y = obj_list.objlist[i].pose.position.y;

			// MAEK TEXT MARKER OF A CUBE MARKER
			text_marker = marker;
				text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				text_marker.text = "object " + std::to_string(i);
				text_marker.pose.position.z = 1.0;
				text_marker.scale.x = 0.0;
				text_marker.scale.y = 0.0;
				text_marker.color.r = 1.0;
				text_marker.color.g = 1.0;
				text_marker.color.b = 1.0;

			// ADD MARKERS TO VISUALIZATION MARKERS ARRAY
			marker_array.markers.push_back(marker);
			text_marker_array.markers.push_back(text_marker);
		}
		// PUBLISH TO SCREEN
		markerarray_pub.publish(marker_array);
		textmarkerarray_pub.publish(text_marker_array);

		// CLEAR ARRAY
		marker_array.markers.clear();
		text_marker_array.markers.clear();
		obj_list.objlist.clear();
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_client_node");

	V2X_Client vc;

	ros::Rate r(1); // 1 Hz

	double start_time, mid_time, end_time;

	while(ros::ok()){
		start_time = ros::Time::now().toSec(); 	// TIME MEASURE START
		ros::spinOnce();
		vc.create_static_objects();
		vc.run();
		end_time = ros::Time::now().toSec(); 	// TIME MEASURE END
		ROS_INFO("time for run is : %f sec",end_time-start_time);
		r.sleep();
	}
	return 0;
}
