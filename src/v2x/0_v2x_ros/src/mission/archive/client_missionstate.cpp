// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <fstream>
// SIGNAL
#include <signal.h>
// SOCKET
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <unistd.h>

#include "obu_message.h"
#include "sample_message.h"

#include "MessageFrame.h"
#include "dsrc_msg_id.h"

#include "libcrc16.h"

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class Call_List_Client {
public:	
	Call_List_Client(void):
		private_nh_("~")	
	{
		initParams();
		initSetup();

		ROS_INFO("CALL LIST CLIENT NODE INITIALIZED");

		///////////////////////////////////// KAIST /////////////////////////////////////
		// struct Mission temp_mission;
		// temp_mission.id = 0;
		// temp_mission.start_xpos = 352921.89;
		// temp_mission.start_ypos = 4026373.79;
		// temp_mission.goal_xpos = 352879.95;
		// temp_mission.goal_ypos = 4026286.19;

		// mission_buf_.emplace_back(temp_mission);

		// temp_mission.id = 1;
		// temp_mission.start_xpos = 352962.59;
		// temp_mission.start_ypos = 4026193.33;
		// temp_mission.goal_xpos = 353071.03;
		// temp_mission.goal_ypos = 4025995.85;
		
		// mission_buf_.emplace_back(temp_mission);
		///////////////////////////////////// KAIST /////////////////////////////////////

		///////////////////////////////////// DAEGU /////////////////////////////////////
		struct Mission temp_mission;
		temp_mission.id = 0;
		temp_mission.start_xpos = 471219.665;
		temp_mission.start_ypos = 3965770.227;
		temp_mission.goal_xpos = 471269.057;
		temp_mission.goal_ypos = 3966025.099;

		mission_buf_.emplace_back(temp_mission);

		temp_mission.id = 1;
		temp_mission.start_xpos = 471715.787;
		temp_mission.start_ypos = 3966108.094;
		temp_mission.goal_xpos = 471842.400;
		temp_mission.goal_ypos = 3965618.173;
		
		mission_buf_.emplace_back(temp_mission);
		///////////////////////////////////// DAEGU /////////////////////////////////////
		
		while (ros::ok()) {	
			if (odom_buf_.size() != 0 && state_buf_.size() != 0) {
				if ((!is_priority_init_) && (is_end_) && (mission_buf_.size() != 0)) checkPriority();
				if ((is_priority_init_) && (mission_buf_.size() != 0)) checkMissionState();
				else ROS_INFO("MISSION FINISHED");
			}	
			ros::spinOnce();
		}
	}

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		private_nh_.getParam("distance_threshold", dist_th_);
		private_nh_.getParam("time_threshold", time_th_);
		is_reach_, is_start_, is_priority_init_, is_time_init_, is_initialized_ = false;
		is_end_ = true;
		sock_, size_, is_connect_ = -1;
		sequence_ = 0;
	}
	void initSetup(void)
	{
		odom_sub_ = nh_.subscribe("gps_odom", 10, &Call_List_Client::odomCallback, this);
		state_sub_ = nh_.subscribe("vehicle_speed", 10, &Call_List_Client::stateCallback, this); 
		start_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
		target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);	
	}
	void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
		odom_buf_.emplace_back(*odom_msg);
		odom_buf_.back().header.stamp = ros::Time::now();
		sleep(0.5);
	}
	void stateCallback(const std_msgs::Float64ConstPtr &state_msg) {
		if (is_reach_) {
			state_buf_.emplace_back(*state_msg);
		}
		sleep(0.5);
	}
	
	std::string static convertArrToHexStr(char *data, int len) {
		std::string s(len * 2, ' ');
		for (int i=0;i<len;++i) {
			s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
			s[2 * i + 1] = hexmap[data[i] & 0x0F];
		}
		return s;
	}
	
	void checkPriority(void) {	
		nav_msgs::Odometry cur_odom = odom_buf_.back();
	       	
		int index = 0;	
		double dist = sqrt(powf(mission_buf_[0].start_xpos - cur_odom.pose.pose.position.x, 2) + powf(mission_buf_[0].start_ypos - cur_odom.pose.pose.position.y, 2));
		
		for (int i=1;i<mission_buf_.size();i++) {
			double temp_dist = sqrt(powf(mission_buf_[i].start_xpos - cur_odom.pose.pose.position.x, 2) + powf(mission_buf_[i].start_ypos - cur_odom.pose.pose.position.y, 2));		
			
			if (temp_dist < dist) {
				dist = temp_dist;
				index = i;
			}
		}	
		
		cur_mission_ = mission_buf_[index];
		is_priority_init_ = true;
	}

	void checkMissionState(void) {
		geometry_msgs::PoseStamped target_msg; 
		nav_msgs::Odometry cur_odom = odom_buf_.back();
		std_msgs::Float64 cur_state = state_buf_.back();

		double dist2start = sqrt(powf(cur_mission_.start_xpos - cur_odom.pose.pose.position.x, 2) + powf(cur_mission_.start_ypos - cur_odom.pose.pose.position.y, 2));	
		double dist2goal = sqrt(powf(cur_mission_.goal_xpos - cur_odom.pose.pose.position.x, 2) + powf(cur_mission_.goal_ypos - cur_odom.pose.pose.position.y, 2));	
		int state = -1;

		if (!is_reach_ && !is_start_) {
			if (!is_initialized_) {
				geometry_msgs::PoseWithCovarianceStamped start_pose_msg;

				start_pose_msg.pose.pose.position.x = cur_odom.pose.pose.position.x;
				start_pose_msg.pose.pose.position.y = cur_odom.pose.pose.position.y;

				start_pose_msg.pose.pose.orientation.x = 0.0;
				start_pose_msg.pose.pose.orientation.y = 0.0;
				start_pose_msg.pose.pose.orientation.z = 0.0;
				start_pose_msg.pose.pose.orientation.w = 1.0;

				start_pose_pub_.publish(start_pose_msg);
				
				target_msg.pose.position.x = cur_mission_.start_xpos;
				target_msg.pose.position.y = cur_mission_.start_ypos;
				target_pub_.publish(target_msg);
				is_initialized_ = true;

				ROS_INFO("GLOBAL PATH 1 PUBLISHED");
			}
			if (dist2start < dist_th_) { 
				// ROS_INFO("ARRIVED AT START POINT");
				is_reach_ = true;
				is_end_ = false;
			}
			// else ROS_INFO("APPROACHING TO START POINT, %f METER LEFT", dist2start);
			state = 0;	
		}
		else if (is_reach_ && !is_start_) {
			if (cur_state.data <= 0.1) {
				if (!is_time_init_) {
					time_ = ros::Time::now();
					ROS_INFO("VEHICLE STOPS AT START POINT");
					is_time_init_ = true;
					is_initialized_ = false;
					if (!is_initialized_) {
						geometry_msgs::PoseWithCovarianceStamped start_pose_msg;

						start_pose_msg.pose.pose.position.x = cur_mission_.start_xpos;
						start_pose_msg.pose.pose.position.y = cur_mission_.start_ypos;

						start_pose_msg.pose.pose.orientation.x = 0.0;
						start_pose_msg.pose.pose.orientation.y = 0.0;
						start_pose_msg.pose.pose.orientation.z = 0.0;
						start_pose_msg.pose.pose.orientation.w = 1.0;

						start_pose_pub_.publish(start_pose_msg);

						target_msg.pose.position.x = cur_mission_.goal_xpos;
						target_msg.pose.position.y = cur_mission_.goal_ypos;
						target_pub_.publish(target_msg);
						is_initialized_ = true;
						
						ROS_INFO("GLOBAL PATH 2 PUBLISHED");
					}
				}
				else {
					double duration = (ros::Time::now() - time_).toSec();
					if (duration > time_th_) {
						ROS_INFO("START CURRENT MISSION", time_th_);
						is_reach_ = false;
						is_start_ = true;
						is_time_init_ = false;
						state_buf_.erase(state_buf_.begin(), state_buf_.end() - 1);
						is_initialized_ = false;
					}
					// else ROS_INFO("WAIT FOR %f MORE SECONDS", time_th_ - duration);
					
				}
			}
			// else ROS_INFO("PLEASE STOP VEHICLE AT START POINT");
			state = 0;	
		}
		else if (!is_reach_ && is_start_) {
			if (dist2goal < dist_th_) {
				ROS_INFO("ARRIVED AT GOAL POINT");
				is_reach_ = true;
			}
			// else ROS_INFO("APPROACHING TO END POINT, %f METER LEFT", dist2goal);
			state = 1;	
		}
		else if (is_reach_ && is_start_) {
			if (cur_state.data <= 0.1) {
				if (!is_time_init_) {
					time_ = ros::Time::now();
					ROS_INFO("VEHICLE STOPS AT GOAL POINT");
					is_time_init_ = true;
				}
				else {
					double duration = (ros::Time::now() - time_).toSec();
					if (duration > time_th_) {
						ROS_INFO("END CURRENT MISSION", time_th_);
						is_reach_ = false;
						is_start_ = false;
						is_end_ = true;
						is_time_init_ = false;
						is_priority_init_ = false;
						state_buf_.erase(state_buf_.begin(), state_buf_.end() - 1);

						for (int i=0;i<mission_buf_.size();i++) {
							if (mission_buf_[i].id == cur_mission_.id) mission_buf_.erase(mission_buf_.begin() + i);
						}
					}
					// else ROS_INFO("WAIT FOR %f MORE SECONDS", time_th_ - duration);
				}
			}
			// else ROS_INFO("PLEASE STOP VEHICLE AT GOAL POINT");
			state = 1;	
		}
		if (state == 0 && !is_initialized_) {
		}
		else if (state == 1 && !is_initialized_) {
		}

		odom_buf_.erase(odom_buf_.begin(), odom_buf_.end() - 1);
	}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_;
	ros::Subscriber state_sub_;
	ros::Publisher start_pose_pub_;
	ros::Publisher target_pub_;
	CRC16 crc16_;

	struct Mission {
		int id;
		double start_xpos;
		double start_ypos;
		double goal_xpos;
		double goal_ypos;
	};

	sockaddr_in obu_addr_;
	std::string obu_ip_;
	int obu_port_, sock_, buff_size_, size_, rate_;
	double dist_th_, time_th_; 
	unsigned char sequence_;
	struct Mission cur_mission_;
	ros::Time time_;

	// BUFFER
	std::vector<nav_msgs::Odometry> odom_buf_;	
	std::vector<std_msgs::Float64> state_buf_;
	std::vector<struct Mission> mission_buf_;

	// CHECK FLAG PARAMETER
	int is_connect_;
	bool is_priority_init_;
	bool is_reach_;
	bool is_start_;
	bool is_end_;
	bool is_time_init_;
	bool is_initialized_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;

	return 0;
}
