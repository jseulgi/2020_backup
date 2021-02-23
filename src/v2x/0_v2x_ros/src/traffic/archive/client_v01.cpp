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
//  NEXT VERSION: ...
//		- 
//		- 
//		- 
//		- 
////////////////////////////////////////

// DEBUGGING 
#define DEBUG 		true		//true: print std::cout, ROS_INFO, ROS_INFO_STREAM for debugging
#define DEBUG_ASN 	true 		//true: print all data information of payload
//#define intrsctn_starting_idx 1001  	//id={1001,1002,1003, ... , 1014}
// #define MARKER	 	false		//true: rviz visualization of traffic objects is turned on

// 다음의 세가지 경우가 있음
#define SOCKET		true 		//true: socket programming is enabled
#define ROSBAG		false 		//true: rosbag file is enabled
#define LOAD_DATA	false		//true: load data is enabled
//



// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
// PATH_MSGS & OBJ_MSGS
#include "path_msgs/Map.h"
#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
// COMMON
#include <iostream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <sstream>
// RVIZ VISUALIZATION
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
//// FOR TESTING ////
#include "obu_message.h"
// #include "sample_message.h"/
// #include "sample_msg_01.h"**


bool is_done_ = false;

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class V2X_Client {
private:
	ros::NodeHandle private_nh_;

	// PUBs AND SUBs
		//pub
	ros::Publisher obj_list_pub = private_nh_.advertise<obj_msgs::ObjList>("object_list",1);

	ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("chatter", 1000);
	
	ros::Publisher markerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("lane_markerArray", 1);
	ros::Publisher textmarkerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("lane_textmarkerArray", 1);
	ros::Publisher traffic_marker_array_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("traffic_markerArray", 1);
	ros::Publisher traffic_text_marker_array_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("traffic_textmarkerArray", 1);
	
		//sub
	ros::Subscriber sub_mapinfo;

	// CLASS VARIABLES (kinda GLOBAL VARIABLE)
	sockaddr_in obu_addr_;
	std::string obu_ip_;
	int obu_port_, sock_, buff_size_, size_, is_debug_on_;
	
	// Used for Decoding 
	MessageFrame_t* msgFrame1 = nullptr; // SPAT Message Frame
	MessageFrame_t* msgFrame2 = nullptr; // MAP Message Frame
	asn_dec_rval_t res1,res2;	
	
	// For Map Translation
	double offset_x = 0; 
	double offset_y = 0;

	// OBJECT LISTS
	obj_msgs::ObjList obj_list;

	// Intersection vs Signal Group and its value: 2D array 
		// number of intersections (Daegu):  14개 id={1001,1002,1003, ... , 1014}
		// number of Signal Groups (표준): maximum 해봤자 6개? 보통은 4개일듯. 그래도 안전하게 10개라고 할까?  
			//	number of signal types: 0~9
	int traffic_signal[14][18][2]; //14 rows of intersections X 10 columns of signals groups X (signal , duration)
	double intersection_lane_xy[14][40][2];    //intersections X lanes의 node[0] X  (x,y)좌표

	// CHECK FLAG PARAMETER
	int is_connect_;

public:
	V2X_Client(void):
		private_nh_("~")
	{
		initParams();
		initSetup();
		ROS_INFO("V2X CLIENT NODE INITIALIZED");
	}

	~V2X_Client(){}

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		sock_, size_, is_connect_ = -1;
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
			sleep(1);
		}
		ROS_INFO("mapinfo subscribed successfully!");
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


	int load_datasamples(){

		std::string hex_spat_payload_str3 = "001380a100302e0ece1a30e3d3d3cad62d2000007ec06100120e8c2d82a752900821800de80200305322c602043001bd0040060a84588060860038e00800c153a9481010c00208010018299163028218004100200305422c4060430008c0040060a9d4a40e08a000dc00800c14c8b1820118001b80100182a1162048228001e002003054ea520a043001180040060a6458c160860023000800c1508b103010c00488010000";
		std::string hex_map_payload_str3 = "0012811028003030541d9c346187a7a79bec600004e34014b00e517b7fd01d7100002d01618028000405000015fbd21764b073b2b7c290a80007486100038601200010090000580ee85a42c281cad38241d20020000c800080002c13942bf1619e256440010800080002c20a42a8961ff655503015000080a00002c5af403116ee99f5f85219000149042000a0c06400020120000b1664ff3c5bb5c7cc8048524005800390001000058af67ec32dd6abdf2800410001000058ac97e152dd5bbd9b8604a0001014000057fa77aec2be92b6480a40e000052104000200144000400015f8c9ed74af706d92018168000405000015d592012ca8d9b047c2907800044851000200061000100005757381102a38bc1818";
		// std::string hex_spat_payload_str3 = "00135c00302e0ece1a30e3d3d3cad64c200004fb0b610017a2b01582a7529008228009600200305422c403043002580040060a9d4a40e08a0035200800c14c8b182010c004ec01001829916305821800dc00200305422c40c043001c200400";
		// std::string hex_map_payload_str3 = "0012815d280030305c1d9c3461c7a7a795ac98c00009f8802960288c96ffa2c54200004b0343005000080800022bfac457016066e1de0b063d13aa58367872f0487400038601200010090004580618ac82c0f24397960c7a27b4b066d0e2e0908480080003200020008b02d712ba581fc88822c19b44a2160c7a1d1c0010800080022c153449616072e2084b060d12565836787780c05400020080004b113703085b55376d42ca423e4082410800283019000080480012c459c02496d249db50b296af902090e4800b0c07400020120004b1106fe5059b0a7b2e2c2e23fd0024352002c0020800080002c3983f1a9726a1cc300024800080002c38bbe6517263dcd540028800080002c367bdd41726a1cd54302d000080a00002bff4bcbd95f081a9c852090000290a200010c0c400020200000afd8cf35857c386a590481c0000800690001000057d5b7a0c2be34b5ca000710001000057c517a552be1c35be00";



		// // SPaT DATA  // //
		std::string payloadstring1 = hex_spat_payload_str3;
		int size_1 = payloadstring1.size()/2;
		unsigned char array_spat[size_1];
		const char * c1 = payloadstring1.c_str();
		// std::cout<<payloadstring1<<std::endl;
		for (int i = 0 ; i < size_1; ++i){
			std::string tmp;
			sscanf(c1,"%2hhx",&array_spat[i]);
			c1+=2;
		}		
		res1 = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame1, array_spat , sizeof(array_spat), 0,0);

		if (res1.code == asn_dec_rval_code_e::RC_WMORE){
			ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
			return 0;
		} else if (res1.code == asn_dec_rval_code_e::RC_FAIL){
			ROS_INFO("FAILED TO DECODE PAYLOAD");
			return 0;
		} else if (res1.code ==asn_dec_rval_code_e::RC_OK ) {
			ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");
			if (msgFrame1->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
				fprintf(stderr,"[recv spat]\n");
				#if DEBUG_ASN
					asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame1);
				#endif
			}
		}


		// // MAP DATA  // //
		std::string payloadstring2 = hex_map_payload_str3;
		int size_2 = payloadstring2.size()/2;
		unsigned char array_map[size_2];
		const char * c2 = payloadstring2.c_str();
		for (int i = 0 ; i < size_2; ++i){
			std::string tmp;
			sscanf(c2,"%2hhx",&array_map[i]);
			c2+=2;
		}
		res2 = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame2, array_map , sizeof(array_map), 0,0);

		if (res2.code == asn_dec_rval_code_e::RC_WMORE){
			ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
			return 0;
		} else if (res2.code == asn_dec_rval_code_e::RC_FAIL){
			ROS_INFO("FAILED TO DECODE PAYLOAD");
			return 0;
		} else if (res2.code ==asn_dec_rval_code_e::RC_OK ) {
			ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");
			if(msgFrame2->messageId == dsrc_msg_id::MAP_DATA){
				fprintf(stderr,"[recv map]\n");
				#if DEBUG_ASN
					asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame2);
				#endif
			}
		}

		return 1;
		
	}


	int run(){
		std::cout<<"\n------------------------------------------------------------------"<<std::endl;
		ROS_INFO("offset_x is : %f",offset_x);
		ROS_INFO("offset_y is : %f",offset_y);

		if (msgFrame1 == nullptr || msgFrame2 == nullptr){
			return -1; 
			// run() next time when msgFrame1 and msgFrame2 are available.
		}

		///////////////////START SPAT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
		auto &spattmp = msgFrame1->value.choice.SPAT; 
		
		for (int i = 0; i < (int)spattmp.intersections.list.count; i++) {
			auto& iti = spattmp.intersections.list.array[i];
			#if DEBUG
				std::cout<<"-----\nintersection index " << i << std::endl;
				std::cout<<"intersection id is : "<<(int)iti->id.id<<std::endl;
			#endif

			for (int j = 0; j < (int)iti->states.list.count; j++){
				auto& itj = iti->states.list.array[j];
				auto& itk = itj->state_time_speed.list.array[0];
			
				#ifdef intrsctn_starting_idx
					int m = (int)iti->id.id-intrsctn_starting_idx;
				#endif
				#ifndef intrsctn_starting_idx
					int m = i;
				#endif
				traffic_signal[m][itj->signalGroup][0] = itk->eventState;
				traffic_signal[m][itj->signalGroup][1] = (double)itk->timing->minEndTime/10;

				#if DEBUG
					std::cout << "-" << std::endl;
					if (itj->signalGroup){std::cout<<"signal group : "<<itj->signalGroup<<std::endl;}						
					std::cout<< "event state is : " << traffic_signal[m][itj->signalGroup][0] << ", which means " <<std::endl;
					if 		(itk->eventState == 0){std::cout << "unavailable" << std::endl;}
					else if (itk->eventState == 1){std::cout << "dark (unlit)" << std::endl;}
					else if (itk->eventState == 2){std::cout << "stop then proceed (flashing red)" << std::endl;}
					else if (itk->eventState == 3){std::cout << "stop and remain (red)" << std::endl;}
					else if (itk->eventState == 4){std::cout << "pre-movement (red+yellow, get ready for green)" << std::endl;}
					else if (itk->eventState == 5){std::cout << "permissive movement allowed (proceed with caution)" << std::endl;}
					else if (itk->eventState == 6){std::cout << "protected movement allowed (proceed)" << std::endl;}
					else if (itk->eventState == 7){std::cout << "permissive clearance (yellow, prepare to stop)" << std::endl;}
					else if (itk->eventState == 8){std::cout << "protected yellow (...?)" << std::endl;}
					else if (itk->eventState == 9){std::cout << "caution conflicting traffic (flashing yellow, proceed with caution)" << std::endl;}
					std::cout<<"signal (probably) ends in "<< traffic_signal[m][itj->signalGroup][1] <<" seconds"<<std::endl;
				#endif
			}

		}
		///////////////////END SPAT////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		

		///////////////////START MAP///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

		
		auto &maptmp = msgFrame2->value.choice.MapData; 

		static int map_count = 0;
		// std::cout<<"hellodd"<<std::endl;
		
		// if(&maptmp.msgIssueRevision != NULL){
		if(maptmp.msgIssueRevision){
			std::cout<<"maptmp.msgIssueRevision is "<< map_count <<std::endl;
			
			std::cout<<"maptmp.msgIssueRevision is "<< (int)maptmp.msgIssueRevision<<std::endl;
			
			// if (map_count == maptmp.msgIssueRevision){ 	return 2;  } //prevents parsing of additional map data 
			map_count = (int)maptmp.msgIssueRevision;
			std::cout<<"maptmp.msgIssueRevision is "<< (int)map_count <<std::endl;
		}

		for (int i=0; i < (int) maptmp.intersections->list.count; i++){
			auto& iti = maptmp.intersections->list.array[i];
			if(iti->id.id){std::cout<<"intersection id is "<<iti->id.id<<std::endl;}
			// std::cout<<"intersection ref position is (lat,long,elev) = ("<<iti->refPoint.lat<<","<<iti->refPoint.Long<<","<<*iti->refPoint.elevation<<")"<<std::endl;

			#ifdef intrsctn_starting_idx
				int m = (int)iti->id.id-intrsctn_starting_idx;
			#endif
			#ifndef intrsctn_starting_idx
				int m = i;
			#endif

			double northing, easting,t_northing, t_easting;
			std::string zone;

			//// MARKERS FOR THE LANE NODES
			visualization_msgs::MarkerArray marker_array;
			visualization_msgs::MarkerArray text_marker_array;
			visualization_msgs::Marker text_marker;
			
			visualization_msgs::Marker marker;
				marker.header.frame_id = "map";
				marker.header.stamp = ros::Time::now();
				marker.ns = "reference point";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::SPHERE; 
				marker.action = visualization_msgs::Marker::ADD;

				t_northing = iti->refPoint.lat;
				t_easting = iti->refPoint.Long;
				t_northing = t_northing/10000000;
				t_easting = t_easting/10000000;
				gps_common::LLtoUTM(t_northing,t_easting, northing, easting, zone);

				ROS_INFO("easting (x coordinate) is : %f",easting);
				ROS_INFO("northing (y coordinate) is : %f",northing);
				// std::cout<<"zone is : "<<zone<<std::endl;

				marker.pose.position.x = easting - offset_x;//iti->refPoint.Long;
				marker.pose.position.y = northing - offset_y;//iti->refPoint.lat;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0;
				marker.pose.orientation.y = 0;
				marker.pose.orientation.z = 0;
				marker.pose.orientation.w = 1;
				marker.lifetime = ros::Duration();
				marker.scale.x = 1.0;
				marker.scale.y = 1.0;
				marker.scale.z = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				marker.color.a = 1.0;

			//// MARKERS FOR THE TRAFFIC SIGN
			visualization_msgs::MarkerArray traffic_marker_array;
			visualization_msgs::MarkerArray traffic_text_marker_array;					
			
			for (int j=0; j < (int)iti->laneSet.list.count; j++){
				auto& itj = iti->laneSet.list.array[j];
				#if DEBUG
					// if(itj->laneID > 2){continue;}
					if(itj->laneID){std::cout<<"-\nlane id is : " << itj->laneID << std::endl;}
				#endif
				
				for (int k=0; k < (int) itj->nodeList.choice.nodes.list.count; k++){
					auto& itk = itj->nodeList.choice.nodes.list.array[k];
					#if DEBUG
						std::cout<<"\tnode XY number: "<<k<<std::endl;
						std::cout<<"\t\tLAT : "<<itk->delta.choice.node_LatLon.lat<<", LONG: "<<itk->delta.choice.node_LatLon.lon<<std::endl;
					#endif
					double tt_northing, tt_easting;
					
					tt_northing = ((double)iti->refPoint.lat  + (double)itk->delta.choice.node_LatLon.lat)/10000000;
					tt_easting = ((double)iti->refPoint.Long + (double)itk->delta.choice.node_LatLon.lon)/10000000;
					gps_common::LLtoUTM(tt_northing,tt_easting, northing, easting, zone);

					marker.ns = "lane" + std::to_string(itj->laneID);
					marker.id = k;
					if (k == 0){ // first node position that will be used to situate markers 
						#ifdef intrsctn_starting_idx
							std::cout<<"(int)iti->id.id is : "<<(int)iti->id.id<<std::endl;
							std::cout<<"intrsctn_starting_idx is : "<<intrsctn_starting_idx<<std::endl;
							std::cout<<"(int)iti->id.id-intrsctn_starting_idx is : "<<(int)iti->id.id-intrsctn_starting_idx<<std::endl;
						#endif
						intersection_lane_xy[m][itj->laneID][0] = easting - offset_x;
						intersection_lane_xy[m][itj->laneID][1] = northing - offset_y;
					}
					marker.pose.position.x = easting - offset_x;
					marker.pose.position.y = northing - offset_y;

					text_marker = marker;
					text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					text_marker.text = "lane" + std::to_string(itj->laneID) + "-" + std::to_string(k);
					text_marker.pose.position.z = 1.0;
					text_marker.scale.x = 0.0;
					text_marker.scale.y = 0.0;
					text_marker.color.r = 1.0;
					text_marker.color.g = 1.0;
					text_marker.color.b = 1.0;
					
					if (j%3 == 0){
						marker.color.r = 1;
						marker.color.g = 0;
						marker.color.b = 0;
					} else if (j%3 == 1){
						marker.color.r = 0;
						marker.color.g = 1;
						marker.color.b = 0;
					} else if (j%3 == 2){
						marker.color.r = 0;
						marker.color.g = 0;
						marker.color.b = 1;
					} 

					// ROS_INFO("northing is : %f",northing);
					// ROS_INFO("easting is : %f",easting);

					marker_array.markers.push_back(marker);
					text_marker_array.markers.push_back(text_marker);
					
				}
			}

			// for each lane make traffic sign
			for (int j=0; j < (int) iti->laneSet.list.count; j++){
				auto& itj = iti->laneSet.list.array[j];
				
				std::cout<<"---\nlane id is : " << itj->laneID << std::endl;
				
				// Traffic Sign with LINE_STRIP //
				visualization_msgs::Marker traffic_marker;
					traffic_marker.header.frame_id = "map";
					traffic_marker.header.stamp = ros::Time::now();
					traffic_marker.ns = "temp_name"; //temp name
					traffic_marker.id = 1000; //temp number
					traffic_marker.type = visualization_msgs::Marker::LINE_LIST; 
					traffic_marker.action = visualization_msgs::Marker::ADD;
					geometry_msgs::Point lane1;
						lane1.x = intersection_lane_xy[m][itj->laneID][0];
						lane1.y = intersection_lane_xy[m][itj->laneID][1];
						lane1.z = 0;

					traffic_marker.pose.position.x = 0;
					traffic_marker.pose.position.y = 0;
					traffic_marker.pose.position.z = 0;

					traffic_marker.pose.orientation.x = 0; //tmp
					traffic_marker.pose.orientation.y = 0; //tmp 
					traffic_marker.pose.orientation.z = 0; //tmp
					traffic_marker.pose.orientation.w = 1; //tmp

					traffic_marker.lifetime = ros::Duration();
					traffic_marker.scale.x = 0.3; //line width
					// traffic_marker.scale.y = 0.5; 
					// traffic_marker.scale.z = 0.5; 

					traffic_marker.color.r = 0.0; 
					traffic_marker.color.g = 1.0; //기본적으로 초록불
					traffic_marker.color.b = 0.0;
					traffic_marker.color.a = 1.0;
			
				// visualization_msgs::Marker text_traffic_marker = traffic_marker;
				// 	text_traffic_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				// 	text_traffic_marker.pose.position.z = 1.0;
				// 	text_traffic_marker.color.a = 1.0;

				if (itj->connectsTo){
					for (int k=0; k<(int)itj->connectsTo->list.count; k++){
						auto& itk = itj->connectsTo->list.array[k];
						if(itk->connectingLane.lane){
							std::cout<<"\tconnects to lane : "<<itk->connectingLane.lane<<std::endl;
							traffic_marker.ns = "from lane " + std::to_string(itj->laneID) + " to lane " + std::to_string(itk->connectingLane.lane);
							traffic_marker.id = k;
							
							traffic_marker.points.push_back(lane1);
							geometry_msgs::Point lane2;
								lane2.x = intersection_lane_xy[m][itk->connectingLane.lane][0];
								lane2.y = intersection_lane_xy[m][itk->connectingLane.lane][1];
								lane2.z = 0;						
							traffic_marker.points.push_back(lane2);

							// MAKE TRAFFIC SIGN OBSTACLE MESSAGE
							static double width = 2; //meters
							static double length = 2; //meters
							static obj_msgs::Obj dummy_object;
							dummy_object.id = 500 + obj_list.objlist.size();
							dummy_object.type = (int16_t) 10; //for traffic sign
							dummy_object.pose.position.x = lane2.x;
							dummy_object.pose.position.y = lane2.y;
							ROS_INFO("object coordinate x : %f", (double)dummy_object.pose.position.x);
							ROS_INFO("object coordinate y : %f", (double)dummy_object.pose.position.y);
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

						if(itk->signalGroup){

							std::cout<<"\t\tSignal group to check is : " << *(itk->signalGroup) <<std::endl;
							std::cout<<"\t\t\tSignal is : " << traffic_signal[m][*(itk->signalGroup)][0] << "-";
							if 		(traffic_signal[m][*(itk->signalGroup)][0] == 0){std::cout << "unavailable" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 1){std::cout << "dark (unlit)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 2){std::cout << "stop then proceed (flashing red)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 3){std::cout << "stop and remain (red)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 4){std::cout << "pre-movement (red+yellow, get ready for green)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 5){std::cout << "permissive movement allowed (proceed with caution)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 6){std::cout << "protected movement allowed (proceed)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 7){std::cout << "permissive clearance (yellow, prepare to stop)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 8){std::cout << "protected yellow (...?)" << std::endl;}
							else if (traffic_signal[m][*(itk->signalGroup)][0] == 9){std::cout << "caution conflicting traffic (flashing yellow, proceed with caution)" << std::endl;}
							std::cout<<"\t\t\tsignal (probably) ends in "<< traffic_signal[m][*(itk->signalGroup)][1] <<" seconds"<<std::endl;

							if 			(traffic_signal[m][*itk->signalGroup][0] == 3){			// RED 
								traffic_marker.color.r = 1.0; 
								traffic_marker.color.g = 0.0; 
								traffic_marker.color.b = 0.0;
							} else if 	((traffic_signal[m][*itk->signalGroup][0] == 5) || 
											(traffic_signal[m][*itk->signalGroup][0] == 6)){			// GREEN
								traffic_marker.color.r = 0.0; 
								traffic_marker.color.g = 1.0; 
								traffic_marker.color.b = 0.0;										
							} else if	(traffic_signal[m][*itk->signalGroup][0] == 7){			// STOPPING YELLOW
								traffic_marker.color.r = 0.0; 
								traffic_marker.color.g = 1.0; 
								traffic_marker.color.b = 1.0;
							} else if 	(traffic_signal[m][*itk->signalGroup][0] == 9){			// BLINKING YELLOW
								traffic_marker.color.r = 0.0; 
								traffic_marker.color.g = 1.0; 
								traffic_marker.color.b = 1.0;
								// how to make it blink?
							} else 		{
								std::cout<<"other SIGNAL CASE: "<< traffic_signal[m][*itk->signalGroup][0] <<std::endl;
								// Do I need to make an counting thing that checks how many other cases are existant?
							}
						}

						traffic_marker_array.markers.push_back(traffic_marker);
						traffic_marker.points.clear();
						// traffic_text_marker_array.markers.push_back(text_traffic_marker);
					}			
				}
			}
			
			markerarray_pub.publish(marker_array);
			textmarkerarray_pub.publish(text_marker_array);
			obj_list_pub.publish(obj_list);		
			marker_array.markers.clear();
			text_marker_array.markers.clear();
			obj_list.objlist.clear();
			
			traffic_marker_array_pub.publish(traffic_marker_array);
			// traffic_text_marker_array_pub.publish(traffic_text_marker_array);
			traffic_marker_array.markers.clear();
			// traffic_text_marker_array.markers.clear();
			
		}
				
		///////////////////END MAP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		return 0;
	}

	void do_recv(){
		// SOCKET INFORMATION CALLBACK
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
						//printing human readable stucture message to stdout
						#if DEBUG_ASN
							asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
						#endif
						
					} else if (msgFrame->messageId == dsrc_msg_id::MAP_DATA){
						ROS_INFO("MAP MESSAGE RECIEVED");
						msgFrame2 = msgFrame;
						//printing human readable stucture message to stdout
						#if DEBUG_ASN
							asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
						#endif
					} else {
						ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
					}	
				} else if (res.code == asn_dec_rval_code_e::RC_WMORE){
					ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
					msgs.erase();
					break;
				} else if (res.code == asn_dec_rval_code_e::RC_FAIL){
					ROS_INFO("FAILED TO DECODE PAYLOAD");
					msgs.erase();
					break;
				}
			}
		}
	}




};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_client_node");

	V2X_Client vc;

	ros::Rate r(10); // 2 Hz

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
		
	double start_time, mid_time, end_time; 

	while(ros::ok()){
		start_time = ros::Time::now().toSec(); 	// TIME MEASURE START
		//// STEP 1
					//update messageFrame1 and messageFrame2 // this doesn't happen for 
					#if ROSBAG
						ros::spinOnce();
					#endif
					#if SOCKET
						vc.do_recv();
					#endif
		mid_time = ros::Time::now().toSec(); 	// TIME MEASURE MID
		//// STEP 2
					int run_result = vc.run();
					if(run_result == -1){
						ROS_ERROR("msgFrame1 or msgFrame2 is empty, another recv() required!");
					} else if(run_result == 2){
						ROS_INFO("same MAP data received");
					} 
		end_time = ros::Time::now().toSec(); 	// TIME MEASURE END

		ROS_INFO("time to recv() is : %f sec and vc.run() is %f sec",mid_time-start_time,end_time-mid_time);
		r.sleep();
	}

	vc.closesocket();

	return 0;
}
