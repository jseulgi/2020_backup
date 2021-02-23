////////////////////////////////////////
// 	VERSION INFO
//	* this version works for the Daegu road.
//		- ros::spinonce() incorporated 				[done]
//		- offset_x, offset_y to match HD MAP 		[done]
//		- ifdef to comment out debugging outputs	[done]
//		- In more detail, this version takes data from the SPaT and MAP payload provided by CEST
// 			and uses this as an input data. The data is processed so all the nodes of each lane
//			in each intersection is a sphere marker. The trajectory linking one node to the other
// 			in green means that the car can proceed in that trajectory. The red one means stop.
//  		This information about the intersection is synced with the Daegu HD map. Objects are
//			also created at the beginning of an intersection node when the light is red so that
//			the car cannot proceed towards that direction.
//	VERSION INFO
//	* this version works also for socket programming.
//		- To take communication data from server, make SOCKET == true
//
//	VERSION 02 INFO
//	* this version is to show how wrong the posiitons of the MAP information are.
//	* With this version you can check
//		- if my green light boolean is correct
//			- check signal group info.
//		- if the obstacles (red light) is accurately created.
//
//	VERSION 03 INFO
//	* this version will only incorporate the SPAT data...
//		- TO DO SO ...
//			1. We need to know which lane listens to which Signal Group. 			[done]
//			2. We need to know which intersection is which intersection id. 		[done]
//			3. We need to save the accurate positions of the red light to create 	[done]
//
//	VERSION 04 INFO
//	* this version takes account of the current location, speed, and the time left to the intersection
//		when making red light signals.
//
//	VERSION 05 INFO
//	* this version dont care about intersections that are too far away when making red light signals.
//		- TO DO SO ...
//			1. Create a list of intersection reference points (교차로의 obstacle의 중점 position을 list로 넣는다)
//	* this version also makes red light singals depending on the path taken
//		- TO DO SO ...
//			1. 
//	* this version also makes only allows right turns when left turn is allowed (비정형에서 빠져나올 때 신호)
//		- TO DO SO ... 
//			1. 모든 우회전은 좌회전과 same 신호를 받게 밑의 리스트를 수정함. 2020-10-07 12:48:41 메일 확인
//
////////////////////////////////////////

// DEBUGGING
#define DEBUG 		false		//true: print std::cout, ROS_INFO, ROS_INFO_STREAM for debugging
#define DEBUG_ASN 	false 		//true: print all data information of payload

// 다음의 세가지 경우가 있음
#define SOCKET		true 		//true: socket programming is enabled
#define ROSBAG		false 		//true: rosbag file is enabled
#define LOAD_DATA	false		//true: load data is enabled
//


// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
// PATH_MSGS & OBJ_MSGS
#include "path_msgs/Map.h"
#include "path_msgs/Trajectory.h"
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
// C++
#include <iostream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <mutex>
#include <utility>
// LLtoUTM
#include "conversions.h"
// SIGNAL
#include <signal.h>
// SOCKET
#include <iomanip>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
// PARSING
#include "MessageFrame.h"
#include "dsrc_msg_id.h"
#include "obu_message.h"


bool is_done_ = false;

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class V2X_Client {
private:
	ros::NodeHandle private_nh_;

	// PUBs AND SUBs
		//pub
	ros::Publisher obj_list_pub = private_nh_.advertise<obj_msgs::ObjList>("/traffic_info",100);
	ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher redlight_marker = private_nh_.advertise<geometry_msgs::PoseArray>("/redlight_marker",100);

		//sub initialy
	ros::Subscriber sub_mapinfo;
		//sub always with ros::spinOnce()
	ros::Subscriber sub_global_path = private_nh_.subscribe<path_msgs::Trajectory>("/global_trajectory", 1, &V2X_Client::linkCallback, this);
	ros::Subscriber odom_sub_ = private_nh_.subscribe<nav_msgs::Odometry>("/gps_odom", 1, &V2X_Client::odomCallback, this);
	#if ROSBAG
		ros::Subscriber sub_rosbag_traffic; //only when using rosbag
	#endif
	
	// CLASS VARIABLES (kinda GLOBAL VARIABLE)
	sockaddr_in obu_addr_;
	std::string obu_ip_ = "192.168.10.10";
	int obu_port_ = 15130;
	int buff_size_ = 3000;
	int sock_, size_;

	// Used for Decoding
	MessageFrame_t* msgFrame1 = nullptr; // SPAT Message Frame
	MessageFrame_t* msgFrame2 = nullptr; // MAP Message Frame
	asn_dec_rval_t res1,res2;

	// For Map Translation
	double offset_x = 0;
	double offset_y = 0;

	// INTERSECTION REFERENCE POINTS
	struct RefPoint{
		int intid;
		double x;
		double y;
	};

	// TRAFFIC STRUCTURES
	struct GroupInfo{
		int no;
		bool greenlight;
		double time_left;
	};

	struct TrafficSignal{
		int intersection_id;
		double timenow;
		std::vector<GroupInfo> signalgroup_buf;
	};

	struct ObstaclePosition {
		int id_intersection;
		int from_lane;
		std::string linkid;
		int signalgroup;
		double x;
		double y;
	};

	// BUFFER
	std::vector<TrafficSignal> trafficsignal_buf;
	std::vector<RefPoint> intersection_ref_buf;
	std::vector<std::string> traffic_payload_buffer;
	std::vector<ObstaclePosition> obstacle_positions_buf;
	std::vector<nav_msgs::Odometry> odom_buf_;
	std::vector<std::string> link_buf_;

	// CHECK FLAG PARAMETER
	int is_connect_;
	bool got_message_;

public:
	V2X_Client(void):
		private_nh_("~")
	{
		initParams();
		initSetup();
		initRedlights();
		initIntRefPoints();
		ROS_INFO("V2X CLIENT NODE INITIALIZED");
	}

	~V2X_Client(){ }

	void initParams(void) {
		sock_, size_, is_connect_ = -1;
		got_message_ = false;
	}

	void initSetup(void)
	{
		initSignalHandler();
		memset(&obu_addr_, 0, sizeof(obu_addr_));
		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_port = htons(uint32_t(obu_port_));

		#if SOCKET
			if (inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr) <= 0) {
				ROS_ERROR("FAILED TO CONVERT IP ADDRESS TO BINARY TYPE");
				ros::shutdown();
			}

			if ((sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
				ROS_ERROR("FAILED TO CREATE SOCKET FILE DESCRIPTOR");
				ros::shutdown();
			}

			if ((is_connect_ = connect(sock_, (const sockaddr *)&obu_addr_, sizeof obu_addr_)) < 0) {
				ROS_ERROR("FAILED TO CONFIGURE TCP/IP SOCKET COMMUNICATION");
				ros::shutdown();
			}
		#endif

		sub_mapinfo = private_nh_.subscribe<path_msgs::Map>("/map_info", 1, &V2X_Client::mapInfonSocketCallback, this);
		while (offset_x == 0 && offset_y == 0){
			ros::spinOnce();
			ROS_ERROR("TURN ON MAP GEN!");
			sleep(1);
		}
		ROS_INFO("mapinfo subscribed successfully!");

		while (odom_buf_.size() == 0){
			ros::spinOnce();
			ROS_ERROR("TURN ON receive gps");
			sleep(1);
		}
		ROS_INFO("gps odom subscribed successfully!");

		#if ROSBAG
		sub_rosbag_traffic = private_nh_.subscribe<std_msgs::String>("/client_v01/chatter", 1000, &V2X_Client::rosbagTrafficCallback, this);
		#endif
	}

	void initRedlights(void){
		// CAUTION: positions are in UTM coordinates
		ObstaclePosition tmp;
		// 형식: tmp ={id_intersection, from_lane, link_id, signalgroup, x,y};

		// 10010
		tmp = {10010,2,"", 8, 471203.398000104,3965716.737999967}; //9
		obstacle_positions_buf.push_back(tmp);
		// 10020
		tmp = {10020,1,"", 8,471348.98300039,3965697.16300017}; //5,12
		obstacle_positions_buf.push_back(tmp);
		tmp ={10020,3,"",10,471370.107000039,3965677.376999992}; //2,12
		obstacle_positions_buf.push_back(tmp);
		tmp ={10020,4,"",10,471369.753000010,3965674.410999995}; //11
		obstacle_positions_buf.push_back(tmp);
		tmp ={10020,9,"",4, 471323.293000001,3965673.711999994}; //6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10020,10,"155M0261ZZ0104",4, 471323.641000022,3965676.663000002}; //5
		obstacle_positions_buf.push_back(tmp);
		tmp ={10020,10,"155M0261LL0104",5, 471323.641000022,3965676.663000002}; //2
		obstacle_positions_buf.push_back(tmp);
		// 10030
		tmp ={10030,1,"",10, 471568.620000024,3965652.499999998}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10030,2,"155M0320LL0104",11, 471568.18400053,3965649.54199948}; // 6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10030,2,"155M0320ZZ0104",10, 471568.184000012,3965649.541999991}; //9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10030,5,"",2, 471543.115000015,3965628.588999973}; //4,9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10030,7,"",4, 471520.073999998,3965649.075999991}; //4,6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10030,8,"",4, 471520.466000044,3965652.020000019}; //3
		obstacle_positions_buf.push_back(tmp);
		// 10040
		tmp ={10040,1,"",10, 471809.789000000,3965622.351999975}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10040,2,"155M0312LL0104",11, 471809.415000006,3965619.366999993}; //6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10040,2,"155M0312ZZ0104",10, 471809.415000003,3965619.366999994}; //9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10040,5,"",2, 471784.307000002,3965598.760999986}; //4,9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10040,7,"",4, 471761.063000018,3965618.880999993}; //4,6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10040,8,"",4, 471761.444000020,3965621.841000012}; //3
		obstacle_positions_buf.push_back(tmp);
		// 10050
		tmp ={10050,1,"",7, 472002.098999989,3965752.283999988}; //13
		obstacle_positions_buf.push_back(tmp);
		tmp ={10050,2,"",7, 472005.083000002,3965752.001999991}; //12
		obstacle_positions_buf.push_back(tmp);
		tmp ={10050,10,"",1, 472006.592000011,3965711.490999994}; //4
		obstacle_positions_buf.push_back(tmp);
		tmp ={10050,11,"" ,1 , 472003.615999979,3965711.916000013}; //3
		obstacle_positions_buf.push_back(tmp);
		// 10160
		tmp ={10160,2,"",7, 472007.112000022,3965996.178000000}; //11
		obstacle_positions_buf.push_back(tmp);
		tmp ={10160,3,"",7, 472009.910000041,3965997.158000016}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10160,8,"",1, 472025.388999986,3965961.972999969}; //5
		obstacle_positions_buf.push_back(tmp);
		tmp ={10160,7,"",1, 472028.322000005,3965962.493999976}; //6
		obstacle_positions_buf.push_back(tmp);
		// 10140
		tmp ={10140,13,"",2, 471960.535999999,3966124.673000018}; //19
		obstacle_positions_buf.push_back(tmp);
		tmp ={10140,16,"",4, 471927.680000013,3966133.026000004}; //15
		obstacle_positions_buf.push_back(tmp);
		// 10170
		tmp ={10170,3,"",10, 471801.738000005,3966118.553000002}; //14
		obstacle_positions_buf.push_back(tmp);
		tmp ={10170,4,"",10, 471801.793999998,3966115.569999998}; //13
		obstacle_positions_buf.push_back(tmp);
		tmp ={10170,10,"",4, 471752.850000034,3966105.249999990}; //7
		obstacle_positions_buf.push_back(tmp);
		tmp ={10170,11,"",4, 471752.805000033,3966108.278999967}; //6
		obstacle_positions_buf.push_back(tmp);
		// 10180
		tmp ={10180,5,"",10, 471628.398000032,3966134.813999998}; //4
		obstacle_positions_buf.push_back(tmp);
		tmp ={10180,6,"",10, 471627.818000031,3966131.859999999}; //3
		obstacle_positions_buf.push_back(tmp);
		tmp ={10180,1,"",4, 471592.902999996,3966131.914999973}; //8
		obstacle_positions_buf.push_back(tmp);
		tmp ={10180,2,"",4, 471593.505000061,3966134.809999998}; //7
		obstacle_positions_buf.push_back(tmp);
		// 10190
		tmp ={10190,3,"",10, 471417.439000011,3966172.029999989}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10190,4,"",10, 471417.643000006,3966169.063999998}; //9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10190,7,"",4, 471386.001000021,3966155.523999974}; //6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10190,8,"",4, 471385.272000006,3966158.434999992}; //5
		obstacle_positions_buf.push_back(tmp);
		// 10200
		tmp ={10200,1,"",7, 471289.202000008,3966082.198999985}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10200,2,"",7, 471292.034000004,3966081.108999993}; //9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10200,8,"",1, 471277.464000023,3966051.070999984}; //3
		obstacle_positions_buf.push_back(tmp);
		tmp ={10200,7,"",1, 471280.210000006,3966049.850999977}; //4
		obstacle_positions_buf.push_back(tmp);
		// 10210
		tmp ={10210,1,"",7, 471250.066000011,3965991.291999987}; //14
		obstacle_positions_buf.push_back(tmp);
		tmp ={10210,2,"",7, 471253.041000010,3965990.861999989}; //13
		obstacle_positions_buf.push_back(tmp);
		tmp ={10210,12,"",1, 471248.716000020,3965947.423000004}; //3
		obstacle_positions_buf.push_back(tmp);
		tmp ={10210,11,"",1, 471251.709000013,3965946.846999997}; //4
		obstacle_positions_buf.push_back(tmp);
		// 10220
		tmp ={10220,1,"",7, 471232.853000014,3965896.419000000}; //10
		obstacle_positions_buf.push_back(tmp);
		tmp ={10220,2,"155M0278LS0104",7, 471235.793000003,3965895.896999997}; //9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10220,2,"155M0278LL0104",8, 471235.793000003,3965895.896999997}; //6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10220,5,"",11, 471255.178000020,3965869.882999989}; //4,9
		obstacle_positions_buf.push_back(tmp);
		tmp ={10220,7,"",1, 471234.239000004,3965850.653999982}; //4,6
		obstacle_positions_buf.push_back(tmp);
		tmp ={10220,8,"",1, 471231.358999992,3965851.356999997}; //3
		obstacle_positions_buf.push_back(tmp);

		// FOR ALL OBSTACLES, CHANGE TO OUR COORDINATE SYSTEM
		for (int i=0; i<obstacle_positions_buf.size(); i++){
			std::cout<<"converting UTM to OUR for "<<obstacle_positions_buf[i].id_intersection<<" lane "<<obstacle_positions_buf[i].from_lane<<std::endl;
			std::vector<double> tt = gpsConv(obstacle_positions_buf[i].x , obstacle_positions_buf[i].y, "UTM","OUR");
			obstacle_positions_buf[i].x = tt[0];
			obstacle_positions_buf[i].y = tt[1];
		}

		// /////// DISPLAY ALL OBSTACLES IN RVIZ TO CHECK ///////
		// ROS_INFO("DISPLAY ALL OBSTACLES IN RVIZ TO CHECK");
		// geometry_msgs::PoseArray check_redlight_buf;
		// check_redlight_buf.header.stamp = ros::Time::now();
		// check_redlight_buf.header.frame_id = "map";
		// for (int i=0; i<obstacle_positions_buf.size(); i++){
		// 	geometry_msgs::Pose redlight;
		// 	redlight.position.x = obstacle_positions_buf[i].x;
		// 	redlight.position.y = obstacle_positions_buf[i].y;
		// 	check_redlight_buf.poses.push_back(redlight);
		// }
		// redlight_marker.publish(check_redlight_buf);
		// sleep(100);
		// ////////////// /////// /////// /////// //////////////

		ROS_INFO("DONE INITALIZING TRAFFIC OBSTACLE POINTS");

	}

	void initIntRefPoints(void){
		RefPoint tmp;
		// 형식: tmp = {intid, x, y}
		tmp = {10010, 471203.544,3965716.744};
		intersection_ref_buf.push_back(tmp);
		tmp = {10020, 471347.201,3965675.981};
		intersection_ref_buf.push_back(tmp);
		tmp = {10030, 471544.578,3965650.382};
		intersection_ref_buf.push_back(tmp);
		tmp = {10040, 471786.188,3965620.289};
		intersection_ref_buf.push_back(tmp);
		tmp = {10050, 472005.51,3965731.51};
		intersection_ref_buf.push_back(tmp);
		tmp = {10160, 472016.65,3965980.98};
		intersection_ref_buf.push_back(tmp);
		tmp = {10140, 471944.957,3966138.358};
		intersection_ref_buf.push_back(tmp);
		tmp = {10170, 471777.226,3966110.499};
		intersection_ref_buf.push_back(tmp);
		tmp = {10180, 471613.016,3966132.892};
		intersection_ref_buf.push_back(tmp);
		tmp = {10190, 471401.763,3966165.534};
		intersection_ref_buf.push_back(tmp);
		tmp = {10200, 471283.389,3966066.188};
		intersection_ref_buf.push_back(tmp);
		tmp = {10210, 471250.747,3965965.686};
		intersection_ref_buf.push_back(tmp);
		tmp = {10220, 471234.662,3965872.491};
		intersection_ref_buf.push_back(tmp);

		// FOR ALL REF POINTS, CHANGE TO OUR COORDINATE SYSTEM
		for (int i=0; i<intersection_ref_buf.size(); i++){
			std::cout<<"converting UTM to OUR for "<<intersection_ref_buf[i].intid<<std::endl;
			std::vector<double> tt = gpsConv(intersection_ref_buf[i].x , intersection_ref_buf[i].y, "UTM","OUR");
			intersection_ref_buf[i].x = tt[0];
			intersection_ref_buf[i].y = tt[1];
		}

		ROS_INFO("DONE INITALIZING INTERSECTION REFERENCE POINTS");
	}

	void closesocket(void){
		ROS_INFO_STREAM("Socket Closed.. terminate this node");
		close(sock_);
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

	std::string static convertArrToHexStr(char *data, int len) {
		std::string s(len * 2, ' ');
		for (int i=0;i<len;++i) {
			s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
			s[2 * i + 1] = hexmap[data[i] & 0x0F];
		}
		return s;
	}

	void mapInfonSocketCallback(const path_msgs::Map msg)
	{
		// RETREIVE MAP OFFSET
		offset_x = msg.OffsetMapX;
		offset_y = msg.OffsetMapY;
	}

	void linkCallback(const path_msgs::Trajectory msg){
		std::mutex here;
		here.lock();
			link_buf_.clear();
			// global trajectory에 있는 Current Link ID를 리스트로 만든다. 
			std::string curr_link = "";
			for (int i=0; i < msg.waypoints.size(); i++){
				if (curr_link != msg.waypoints[i].CLID){
					std::cout<<"curr_link : "<<curr_link;
					std::cout<<" | next_link : "<<msg.waypoints[i].CLID<<std::endl;
					link_buf_.push_back(msg.waypoints[i].CLID);
					curr_link = msg.waypoints[i].CLID;
				}
			}
		here.unlock();		
	}

	void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
		std::mutex here;
		here.lock();
			if (odom_buf_.size() > 100) odom_buf_.clear();
			odom_buf_.emplace_back(*odom_msg);
			odom_buf_.back().header.stamp = ros::Time::now();
		here.unlock();
	}

	#if ROSBAG
	// CALLBACK if we are using a bag file of traffic information
	void rosbagTrafficCallback(const std_msgs::String msg){
		std::cout<<msg.data<<std::endl;
		std::string new_msg = msg.data;
				// traffic_payload_buffer.emplace_back(msg.data);
				// if (traffic_payload_buffer.size() > 500){
				// 	traffic_payload_buffer.clear();
				// }
				// BUFFER 방식 안쓸듯!

				// unsigned char * tmp = string2uint8array(new_msg);
		int size = new_msg.size()/2;
		unsigned char array[size];
		const char * c = new_msg.c_str();
		for (int i=0; i<size; ++i){
			std::string tmp;
			sscanf(c,"%2hhx",&array[i]);
			c+=2;
		}

		// DECODING PAYLOAD
		MessageFrame_t* msgFrame = nullptr;
		auto res = uper_decode(0, &asn_DEF_MessageFrame, (void**)&msgFrame, array, sizeof(array), 0, 0);

		if (res1.code == asn_dec_rval_code_e::RC_WMORE){
			ROS_ERROR("MORE DATA REQUIRED, PLEASE CALL AGAIN");
			got_message_ = false;
		} else if (res1.code == asn_dec_rval_code_e::RC_FAIL){
			ROS_ERROR("FAILED TO DECODE PAYLOAD");
			got_message_ = false;
		} else if (res1.code ==asn_dec_rval_code_e::RC_OK ) {
			ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");
			if (msgFrame->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
				ROS_INFO("SPAT MESSAGE RECEIVED");
				msgFrame1 = msgFrame;
				got_message_ = true;
				//printing human readable stucture message to stdout
				#if DEBUG_ASN
				// if (msgFrame1->value.choice.SPAT.intersections.list.array[0]->id.id == 10010){
				// 	asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				// }
				asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				#endif
			} else {
				ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
			}

		}

	}
	#endif

	// unsigned char * string2uint8array(std::string &thisone){
	// 	int size = thisone.size()/2;
	// 	unsigned char array[size];
	// 	const char * c = thisone.c_str();
	// 	for (int i=0; i<size; ++i){
	// 		std::string tmp;
	// 		sscanf(c,"%2hhx",&array[i]);
	// 		c+=2;
	// 	}
	// 	return array;
	// }

	void do_recv(){
		// GET INFORMATION FROM SOCKET
		char raw_data[buff_size_];
		std::string msgs;

		if (((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) > 0)) {

			ROS_INFO("CORRECT PACKET RECEIVED, [CURRENT: %d Bytes / MINIMUM: %d Bytes]", size_, (int)sizeof(obu_tcp_header_t));
			msgs.append((char *)raw_data, size_);

			while (msgs.size() != 0) {
				std::string payload;
				obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];
				payload.append(msgs, sizeof(obu_tcp_header_t), header->payload_size);

				#if DEBUG
					ROS_INFO("[MESSAGE TYPE: 0x%x]", header->packet_type);
					ROS_INFO("[SEQUENCE: %d]", header->current_sequence);
					ROS_INFO("[PAYLOAD SIZE: %d]", header->payload_size);
					ROS_INFO("[DEVICE TYPE: 0x%X]", header->device_type);
					ROS_INFO("[DEVICE ID: %02X-%02X-%02X]", header->device_id[2], header->device_id[1], header->device_id[0]);
					ROS_INFO("[PAYLOAD: 0x%s]", convertArrToHexStr((char*)payload.c_str(), payload.size()).c_str());
				#endif

				// FOR ROS PUBLISHING //
				std_msgs::String ros_msgs;
				std::string hello = convertArrToHexStr((char*)payload.c_str(), payload.size()).c_str();
				ros_msgs.data = hello;
				chatter_pub.publish(ros_msgs);

				// DECODING PAYLOAD
				MessageFrame_t* msgFrame = nullptr;
				auto res = uper_decode(0, &asn_DEF_MessageFrame, (void**)&msgFrame, payload.c_str(), payload.size(), 0, 0);

				if (res.code ==asn_dec_rval_code_e::RC_OK ) {
					ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");
					msgs.erase(0, sizeof(obu_tcp_header_t) + payload.size());

					if  (msgFrame->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
						ROS_INFO("SPAT MESSAGE RECIEVED");
						msgFrame1 = msgFrame;
						got_message_ = true;
						//printing human readable stucture message to stdout
						#if DEBUG_ASN
							asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
						#endif

					} else {
						ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
					}
				} else if (res.code == asn_dec_rval_code_e::RC_WMORE){
					ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
					got_message_ = false;
					msgs.erase();
					break;
				} else if (res.code == asn_dec_rval_code_e::RC_FAIL){
					ROS_INFO("FAILED TO DECODE PAYLOAD");
					got_message_ = false;
					msgs.erase();
					break;
				}
			}
		}
	}


	int run(){

		std::cout<<"------- STEP 2: run()------------------------------------------------------"<<std::endl;

		if ((got_message_ == true)&& (msgFrame1 != nullptr)){
			TrafficSignal tmp_signal;
			///////////////////START SPAT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			auto& spattmp = msgFrame1->value.choice.SPAT;
			auto& iti = spattmp.intersections.list.array[0];

			#if DEBUG
				std::cout<<"signal id is : "<<(int)iti->id.id<<std::endl;
			#endif

			tmp_signal.intersection_id = (int)iti->id.id;
			tmp_signal.timenow = ros::Time::now().toSec();
			

			// for each signal group of a Intersection
			for (int j = 0; j < (int)iti->states.list.count; j++){
				auto& itj = iti->states.list.array[j];
				auto& itk = itj->state_time_speed.list.array[0];
				struct GroupInfo tmp_group;
				tmp_group.no = itj->signalGroup;
				tmp_group.time_left = (double) itk->timing->minEndTime;
				tmp_group.time_left = tmp_group.time_left/10;
				int eventstate = (int) itk->eventState;
				tmp_group.greenlight = greenjudger(eventstate,tmp_group.no,tmp_group.time_left,tmp_signal.intersection_id, odom_buf_.back());
				#if DEBUG
					std::cout<<"Signal Group: "<<tmp_group.no <<std::endl;
					if (tmp_group.greenlight == true){
						std::cout<<"green light: true"<<std::endl;
					} else {
						std::cout<<"green light: false"<<std::endl;
					}
				#endif
				tmp_signal.signalgroup_buf.push_back(tmp_group);
			}

			int whatisthis = updatespat(tmp_signal);
			std::cout<<"update result (0: updated, 1: newly added) is : "<<whatisthis<<std::endl;

			msgFrame1 = nullptr;
			///////////////////END SPAT////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		} else {
			return -1;
		}

		return 0;
	}

	bool greenjudger(int eventState, int signalGroup, double minEndTime, int intersection_id, nav_msgs::Odometry currodom){
		ROS_INFO("Judging for greenlight");
		// return true -> 녹색등 ; return false -> 적색등
		if (eventState == 0){
			// 점멸 신호등일 때
			return true;
		} else if (eventState != 5 && eventState != 6){
			// 녹색등이 아닐 때
			return false;
		} else {
			// 녹색등일 때
			if (minEndTime > 10){
				// time이 많이 남았으면
				return true;
			} else {
				ROS_WARN("LESS THAN 10 SECONDS LEFT");
				// time이 얼마 남지 않은 상황에서
				// 적색등으로 미리 바꿔놓을지 말지 판단

				// STEP 1: 거리 계산
				double obj_x, obj_y;
				for (int i=0; i < obstacle_positions_buf.size();i++){
					if (obstacle_positions_buf[i].id_intersection == intersection_id){
						if (obstacle_positions_buf[i].signalgroup == signalGroup){
							// same 교차로이고 same signalGroup인 node를 하나만 찾아도됨
							// reason: 위 조건을 만족하는 node들이 가깝기 때문에
							obj_x = obstacle_positions_buf[i].x;
							obj_y = obstacle_positions_buf[i].y;
							break;
						}
					}
				}
				// if (obj_x == 0 && obj_y == 0){
				// 	// signal group that we dont care about
				// 	return true;
				// 	// 이래도 되나?.......
				// }
				double dist = lindist(currodom.pose.pose.position.x, currodom.pose.pose.position.y, obj_x, obj_y);
				ROS_WARN("FOR INTERSECTION %d, signal group %d",intersection_id,signalGroup);
				ROS_WARN("(%f,%f) to (%f,%f)",currodom.pose.pose.position.x, currodom.pose.pose.position.y, obj_x, obj_y);
				ROS_WARN("distance to signal is %f meters", dist);
				// STEP 2: 거리 조건 확인하기
				if (dist < 20){
					//거리가 가깝기 때문에 green으로 유지 (보이는 신호 그대로 씀)
					return true;
				} else {
					//거리가 멀기 때문에 red로 전환
					return false;
				}
			}
		}
	}

	int updatespat(const TrafficSignal &info){
		// CASE 1: return 0 --- if id existant in map form, update spat data
		ROS_INFO("spat id is : %d",info.intersection_id);
		for (int i=0; i < trafficsignal_buf.size(); i++){
			if (trafficsignal_buf[i].intersection_id == info.intersection_id){ //same id를 찾는다면?
				trafficsignal_buf[i] = info;
				return 0; // update successful
			}
		}

		// CASE 2: return 1 --- if id not existant, update by adding this intersection to trafficsignal_buf
		trafficsignal_buf.push_back(info);
		return 1; // new intersection signal information
	}


	void publishObjects(){
		// 현재에 each 지점에 red light인지 아닌지 판단을 도와주는 obstacle 만들기.
		std::cout<<"------- STEP 3: PUBLISHING OBJECTS------"<<std::endl;

		geometry_msgs::PoseArray redlight_buf;
		redlight_buf.header.stamp = ros::Time::now();
		redlight_buf.header.frame_id = "map";

		// FOR EACH INTERSECTION (13개)
		for (int i=0; i < trafficsignal_buf.size(); i++) {
			auto& this_intersection = trafficsignal_buf[i];
			#if DEBUG
				std::cout<<"-"<<std::endl;
				std::cout<<"intersection number: "<<trafficsignal_buf[i].intersection_id<<std::endl;

				// CHECK STRUCTURE OF THIS INTERSECTION
				print_structure(this_intersection);
			#endif

			// #if SOCKET
			// // FILTER 1: only use SPAT data which are received within 20 seconds.
			// if (this_intersection.signal.timenow + 20 > ros::Time::now().toSec()){continue;}
			// #endif
			
			// FILTER 2: only use intersection information that are close 
			double f_dist = 0;
			for (int j=0; j < intersection_ref_buf.size(); j++){
				if (this_intersection.intersection_id == intersection_ref_buf[j].intid){
					double f_dist = lindist(intersection_ref_buf[j].x,intersection_ref_buf[j].y, odom_buf_.back().pose.pose.position.x, odom_buf_.back().pose.pose.position.y);
				}	
			}
			if (f_dist > 100){
				continue; //skip adding obstalce for this intersection
			}
			

			// FOR EACH NON-GREEN LIGHT SIGNAL ( red,yellow,or  almost red light )
			for (int j=0; j < this_intersection.signalgroup_buf.size(); j++){
				auto& this_signalgroup = this_intersection.signalgroup_buf[j];
				// + green light 표기가 아니라면... (즉, 적색등이면)
				if (this_signalgroup.greenlight == false){
					// 해당 intersection id와 signal group으로 어디에 object를 만들어야하는지 결정하기.
					for (int k=0; k < obstacle_positions_buf.size(); k++){
						// + 신경쓸 필요 없는 linkid라면 ... (즉, 볼 필요없는 신호등이라면) 
						if ( is_invalidlink(obstacle_positions_buf[k].linkid) ){
							// 이 object는 스킵하기
							continue;
						}

						if ((obstacle_positions_buf[k].id_intersection == this_intersection.intersection_id)
							&& (this_signalgroup.no ==  obstacle_positions_buf[k].signalgroup)){
								geometry_msgs::Pose redlight;
								redlight.position.x = obstacle_positions_buf[k].x;
								redlight.position.y = obstacle_positions_buf[k].y;
								redlight_buf.poses.push_back(redlight);
							
						}

					}
				}
			}

		}

		redlight_marker.publish(redlight_buf);

		// TRAFFIC OBJECT LISTS
		obj_msgs::ObjList traffic_obj_list;
		
		for (int i=0; i <redlight_buf.poses.size();i++){
			obj_msgs::Obj tmp;
			tmp.pose = redlight_buf.poses[i];
			traffic_obj_list.objlist.push_back(tmp);
		}
		obj_list_pub.publish(traffic_obj_list);
	}

	bool is_invalidlink(std::string linkid){
		// 'linkid'가 비어 있지도 않고, trajectory에도 없는 linkid라면 invalid한 link니까 return true;
		if (linkid == ""){
			return false;
		} else {
			for (int i=0;i<link_buf_.size();i++){
				// 내가 가는 link path에 이 link가 있다면 
				if(link_buf_[i] == linkid) return false;
			}
		}

		return true; 
	}

	void print_structure(struct TrafficSignal &intersection){
		//ㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁ
		// if (intersection.intersection_id != 10010) return;
		//ㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁ
		std::cout<<"id: "<<intersection.intersection_id<<std::endl;
		for (int i=0; i<intersection.signalgroup_buf.size();i++){
			std::cout<<"\tSignal Group: "<<intersection.signalgroup_buf[i].no<<std::endl;
			std::cout<<"\tgreen light?: "<<(bool)intersection.signalgroup_buf[i].greenlight<<std::endl;
			std::cout<<"\ttime left: "<<intersection.signalgroup_buf[i].time_left<<std::endl;
		}
	}

	double lindist(double &x1, double &y1, double &x2, double &y2){
		double dx = x1 - x2;
		double dy = y1 - y2;
		return sqrt(dx*dx+dy*dy);
	}

	std::vector<double> gpsConv(const double latxe, const double longyn, const std::string source, const std::string dest){
		// ROS_INFO("From |%s| to |%s|",source.c_str(),dest.c_str());
		// ROS_INFO("From %s(x,y) = (%f,%f)",source.c_str(),latxe,longyn);
		// sleep(3);
		std::vector<double> positionxy;
		double x, y;
		const std::string our_zone = "52N";
		std::string zone;

		if (source == "LL" && dest == "UTM"){
			gps_common::LLtoUTM(latxe,longyn, y, x, zone);
		}else if (source == "LL" && dest == "OUR"){
			gps_common::LLtoUTM(latxe,longyn, y, x, zone);
			x = x - offset_x;
			y = y - offset_y;
		} else if (source == "OUR" && dest == "LL"){
			double tx = latxe + offset_x;
			double ty = longyn + offset_y;
			gps_common::UTMtoLL(ty,tx,our_zone,x,y);
		} else if (source == "OUR" && dest == "UTM"){
			x = latxe + offset_x;
			y = longyn + offset_y;
		} else if (source == "UTM" && dest == "LL"){
			// ROS_INFO_STREAM("our_zone: "<<our_zone);
			gps_common::UTMtoLL(longyn,latxe,our_zone,x,y);
		} else if (source == "UTM" && dest == "OUR"){
			x = latxe - offset_x;
			y = longyn - offset_y;
		} else{
			ROS_ERROR("WRONG INPUT CONVERSION SETTING| FROM %s to %s DOESNT EXIST",source.c_str(), dest.c_str());
			ros::shutdown();
		}

		positionxy.push_back(x);
		positionxy.push_back(y);

		// ROS_INFO("To %s(x,y) = (%f,%f)",dest.c_str(),positionxy[0],positionxy[1]);

		return positionxy;
	}

	// ANY NEW FUNCTIONS? PUT HERE!



};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_client_node");

	V2X_Client vc;

	ros::Rate r(10); // Hz

	#if SOCKET
		ROS_ERROR("starting SOCKET ");
	#endif
	#if ROSBAG
		ROS_ERROR("starting ROSBAG ");
	#endif
	#if LOAD_DATA
		if( vc.load_datasamples() == 0 ){
			ROS_ERROR("PARSING ERROR! RERUN THIS NODE");
		}
	#endif

	double start_time, mid_time, end_time, finish_time;

	while(ros::ok()){
		//// STEP 0
			// odom_buf & link_buf 채우기
		ros::spinOnce();
		start_time = ros::Time::now().toSec(); 		// TIME MEASURE START
		//// STEP 1
					//update messageFrame1 and messageFrame2 // this doesn't happen for
					#if SOCKET
						vc.do_recv();
					#endif
		mid_time = ros::Time::now().toSec(); 		// TIME MEASURE MID
		//// STEP 2
					int run_result = vc.run();
					if(run_result == -1){
						ROS_WARN("spat not received, another recv() required!"); //원래: msgFrame1 is empty, another recv() required!
					} else if (run_result == -2){
						ROS_WARN("intersection is far. Don't care about this data");
					}
		end_time = ros::Time::now().toSec(); 		// TIME MEASURE END
		//// STEP 3
					vc.publishObjects();
		finish_time = ros::Time::now().toSec(); 	// TIME MEASURE FINISH

		ROS_INFO("[MAIN LOOP] time to recv() is : %f sec and vc.run() is %f sec and vc.publishObjects() is %f secs\ntotal time is %f sec\n\n",mid_time-start_time,end_time-mid_time, finish_time-end_time,finish_time-start_time);
		r.sleep();
	}

	vc.closesocket();

	return 0;
}
