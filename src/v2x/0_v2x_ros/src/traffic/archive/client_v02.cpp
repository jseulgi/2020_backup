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
//	VERSION INFO
//	* this version is to show how wrong the posiitons of the MAP information are.
//	* With this version you can check
//		- if my green light boolean is correct
//			- check signal group info.
//		- if the obstacles (red light) is accurately created.
//	* 문제점: 현재 lane의 "connectsTo"가 없어도 있다고 인식되는 문제가 있음. (코드 오류일듯)
//
//	
// 	
//	NEXT VERSION
//	* the next version will only incorporate the SPAT data...
//		- TO DO SO ...
//			1. We need to know which lane listens to which Signal Group.
//			2. We need to know which intersection is which intersection id.
//
//	
//
//
//
//
//
//
//
////////////////////////////////////////

// DEBUGGING
#define DEBUG 		false		//true: print std::cout, ROS_INFO, ROS_INFO_STREAM for debugging
#define DEBUG_ASN 	true 		//true: print all data information of payload
#define MARKER	 	false		//true: rviz visualization of traffic objects is turned on

// 다음의 세가지 경우가 있음
#define SOCKET		false 		//true: socket programming is enabled
#define ROSBAG		true 		//true: rosbag file is enabled
#define LOAD_DATA	false		//true: load data is enabled
//


// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
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
	ros::Publisher obj_list_pub = private_nh_.advertise<obj_msgs::ObjList>("object_list",1);
	ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher object_marker = private_nh_.advertise<geometry_msgs::PoseArray>("object_marker",100);

		//sub
	ros::Subscriber sub_mapinfo;
	ros::Subscriber sub_rosbag_traffic;

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

	// TRAFFIC STRUCTURES
	struct GroupInfo{
		int no;
		bool greenlight;
		double time_left;
	};

	struct SignalInfo{
		int id;
		double timenow;
		std::vector<GroupInfo> signalgroup_buf;
	};

	struct LaneInfo{
		int no;
		double our_x;
		double our_y;
		//connects to which lane and its signal group
		int signaltowatch;
		// std::vector<int,int> connectsTo;
	};

	struct IntersectionInfo{
		int id;
		std::vector<LaneInfo> lane_buf;
	};

	struct TrafficSignal {
		int id;
		SignalInfo signal;
		IntersectionInfo intersection;
	};

	// BUFFER
	std::vector<TrafficSignal> trafficsignal_buf;
	std::vector<std::string> traffic_payload_buffer;

	// CHECK FLAG PARAMETER
	int is_connect_;
	std::string spat_or_map = "nothing here yet";

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

		#if ROSBAG
		sub_rosbag_traffic = private_nh_.subscribe<std_msgs::String>("/client_v01/chatter", 1000, &V2X_Client::rosbagTrafficCallback, this);
		#endif
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
			spat_or_map = "error";
		} else if (res1.code == asn_dec_rval_code_e::RC_FAIL){
			ROS_ERROR("FAILED TO DECODE PAYLOAD");
			spat_or_map = "error";
		} else if (res1.code ==asn_dec_rval_code_e::RC_OK ) {
			ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");
			if (msgFrame->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
				ROS_INFO("SPAT MESSAGE RECEIVED");
				msgFrame1 = msgFrame;
				spat_or_map = "spat";
				//printing human readable stucture message to stdout
				#if DEBUG_ASN
				// if (msgFrame1->value.choice.SPAT.intersections.list.array[0]->id.id == 10010){
				// 	asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				// }
				asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				#endif
			} else if(msgFrame->messageId == dsrc_msg_id::MAP_DATA){
				ROS_INFO("MAP MESSAGE RECEIVED");
				msgFrame2 = msgFrame;
				spat_or_map = "map";
				//printing human readable stucture message to stdout
				#if DEBUG_ASN
				// if (msgFrame2->value.choice.MapData.intersections->list.array[0]->id.id == 10010){					
				// 	asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				// }
				asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
				#endif
			} else {
				ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
			}

		}

	}

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

	int load_datasamples(){

		// std::string hex_spat_payload_str3 = "001380a100302e0ece1a30e3d3d3cad62d2000007ec06100120e8c2d82a752900821800de80200305322c602043001bd0040060a84588060860038e00800c153a9481010c00208010018299163028218004100200305422c4060430008c0040060a9d4a40e08a000dc00800c14c8b1820118001b80100182a1162048228001e002003054ea520a043001180040060a6458c160860023000800c1508b103010c00488010000";
		// std::string hex_map_payload_str3 = "0012811028003030541d9c346187a7a79bec600004e34014b00e517b7fd01d7100002d01618028000405000015fbd21764b073b2b7c290a80007486100038601200010090000580ee85a42c281cad38241d20020000c800080002c13942bf1619e256440010800080002c20a42a8961ff655503015000080a00002c5af403116ee99f5f85219000149042000a0c06400020120000b1664ff3c5bb5c7cc8048524005800390001000058af67ec32dd6abdf2800410001000058ac97e152dd5bbd9b8604a0001014000057fa77aec2be92b6480a40e000052104000200144000400015f8c9ed74af706d92018168000405000015d592012ca8d9b047c2907800044851000200061000100005757381102a38bc1818";
		std::string hex_spat_payload_str3 = "00135c00302e0ece1a30e3d3d3cad64c200004fb0b610017a2b01582a7529008228009600200305422c403043002580040060a9d4a40e08a0035200800c14c8b182010c004ec01001829916305821800dc00200305422c40c043001c200400";
		std::string hex_map_payload_str3 = "0012815d280030305c1d9c3461c7a7a795ac98c00009f8802960288c96ffa2c54200004b0343005000080800022bfac457016066e1de0b063d13aa58367872f0487400038601200010090004580618ac82c0f24397960c7a27b4b066d0e2e0908480080003200020008b02d712ba581fc88822c19b44a2160c7a1d1c0010800080022c153449616072e2084b060d12565836787780c05400020080004b113703085b55376d42ca423e4082410800283019000080480012c459c02496d249db50b296af902090e4800b0c07400020120004b1106fe5059b0a7b2e2c2e23fd0024352002c0020800080002c3983f1a9726a1cc300024800080002c38bbe6517263dcd540028800080002c367bdd41726a1cd54302d000080a00002bff4bcbd95f081a9c852090000290a200010c0c400020200000afd8cf35857c386a590481c0000800690001000057d5b7a0c2be34b5ca000710001000057c517a552be1c35be00";


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
						spat_or_map = "spat";
						//printing human readable stucture message to stdout
						#if DEBUG_ASN
							asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
						#endif

					} else if (msgFrame->messageId == dsrc_msg_id::MAP_DATA){
						ROS_INFO("MAP MESSAGE RECIEVED");
						msgFrame2 = msgFrame;
						spat_or_map = "map";
						//printing human readable stucture message to stdout
						#if DEBUG_ASN
							asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);
						#endif
					} else {
						ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
					}
				} else if (res.code == asn_dec_rval_code_e::RC_WMORE){
					ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
					spat_or_map = "error";
					msgs.erase();
					break;
				} else if (res.code == asn_dec_rval_code_e::RC_FAIL){
					ROS_INFO("FAILED TO DECODE PAYLOAD");
					spat_or_map = "error";
					msgs.erase();
					break;
				}
			}
		}
	}


	int run(){

		std::cout<<"------- STEP 2: run()------------------------------------------------------"<<std::endl;

		if (spat_or_map == "spat"){
			SignalInfo tmp_signal;
			///////////////////START SPAT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			auto& spattmp = msgFrame1->value.choice.SPAT;
			auto& iti = spattmp.intersections.list.array[0];

			#if DEBUG
				std::cout<<"signal id is : "<<(int)iti->id.id<<std::endl;
			#endif

			tmp_signal.id = (int)iti->id.id;
			tmp_signal.timenow = ros::Time::now().toSec();

			// for each signal group of a Intersection
			for (int j = 0; j < (int)iti->states.list.count; j++){
				auto& itj = iti->states.list.array[j];
				GroupInfo tmp_group;
				tmp_group.no = itj->signalGroup;
				std::cout<<"Signal Group: "<<tmp_group.no <<std::endl;
				auto& itk = itj->state_time_speed.list.array[0];
				if (itk->eventState == 5 || itk->eventState == 6 ){
					tmp_group.greenlight = true;
					std::cout<<"green light: true"<<std::endl;
				} else {
					tmp_group.greenlight = false;
					std::cout<<"green light: false"<<std::endl;
				}
				tmp_group.time_left = (double) itk->timing->minEndTime/10;

				tmp_signal.signalgroup_buf.push_back(tmp_group);
			}

			int whatisthis = updatespat(tmp_signal);
			std::cout<<"update result (0: successful, 1: no MAP available) is : "<<whatisthis<<std::endl;

			///////////////////END SPAT////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		} else if (spat_or_map == "map"){

			///////////////////START MAP///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			auto &maptmp = msgFrame2->value.choice.MapData;
			auto& iti = maptmp.intersections->list.array[0];
			std::cout<<"intersection id is "<<iti->id.id<<std::endl;


			IntersectionInfo tmp_intersection;
			tmp_intersection.id = iti->id.id;

			// early return to save computation
			if ( updatemap(tmp_intersection,"dontupdate") == 0 ){
				return 3;
			}

			double northing, easting,t_northing, t_easting;
			std::string zone;

			///// For each lane of an intersection
			for (int j=0; j < (int)iti->laneSet.list.count; j++){
				auto& itj = iti->laneSet.list.array[j];
				auto& itk = itj->nodeList.choice.nodes.list.array[0];
				#if DEBUG
					if(itj->laneID){std::cout<<"-\nlane id is : " << itj->laneID << std::endl;}
				#endif

				double tt_northing, tt_easting;

				tt_northing = ((double)iti->refPoint.lat  + (double)itk->delta.choice.node_LatLon.lat)/10000000;
				tt_easting = ((double)iti->refPoint.Long + (double)itk->delta.choice.node_LatLon.lon)/10000000;
				gps_common::LLtoUTM(tt_northing,tt_easting, northing, easting, zone);

				LaneInfo tmp_lane;
				tmp_lane.no = itj->laneID;				
				tmp_lane.our_x = easting - offset_x;				
				tmp_lane.our_y = northing - offset_y;
				if(itj->connectsTo){
					auto& itk = itj->connectsTo->list.array[0];
					if (itk->signalGroup){
						tmp_lane.signaltowatch = (int) *(itk->signalGroup);
					}
				}
				// 	for (int k=0; k<(int)itj->connectsTo->list.count; k++){
				// 		auto& itk = itj->connectsTo->list.array[k];
				// 		tmp_lane.connectsTo.push_back({(int)itk->connectingLane.lane, });
				// 	}
				tmp_intersection.lane_buf.push_back(tmp_lane);
			}

			int whatisthis = updatemap(tmp_intersection, "update");
			std::cout<<"update result (0: MAP already available, 1: MAP updated) is : "<<whatisthis<<std::endl;
			///////////////////END MAP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		} else {
			ROS_ERROR("run() next time when spat_or_map (:%s) is correctly recieved", spat_or_map.c_str());
			return -1;
		}

		return 0;
	}

	int updatespat(const SignalInfo &info){ 
		// return 주로 0 임

		// CASE 1: return 0 --- if id existant in map form, update spat data
		ROS_INFO("spat id is : %d",info.id);
		for (int i=0; i < trafficsignal_buf.size(); i++){
			if (trafficsignal_buf[i].id == info.id){ //same id를 찾는다면?
				trafficsignal_buf[i].signal = info;
				return 0; // update successful
			}
		}

		// CASE 2: return 1 --- if id not existant in map form, skip this update
		return 1; // not updated

	}

	int updatemap(const IntersectionInfo &newdata,const std::string updatepermission){
		// return 주로 0임
		
		// CASE 1: return 0 --- 이미 특정 intersection 정보를 가지고 있다면 update하지 않기
		for (int i=0; i < trafficsignal_buf.size(); i++){
			std::cout<<"newdata.id ?= trafficsignal_buf[i].id  :  ";
			std::cout<<newdata.id << " ?= ";
			std::cout<<trafficsignal_buf[i].id<<std::endl;
			if ( trafficsignal_buf[i].intersection.id == newdata.id ){
				return 0;
			}
		}
		// CASE 2: if this function reached here, that means that this is a new MAP data, 
		if (updatepermission == "dontupdate"){
			return 10;
		}

		//////////////////////// ////////////////////////
		if (newdata.id != 10010){return 11;}
		//////////////////////// ////////////////////////

		// CASE 3: return 1 --- update를 했음
		struct TrafficSignal t;
		t.id = newdata.id;
		t.intersection = newdata;
		for (int i=0; i < newdata.lane_buf.size(); i++){
			ROS_ERROR("just checking");
			std::cout<<"\tlane: "<< newdata.lane_buf[i].no<<std::endl;
			std::cout<<"\tsignal to watch: "<< newdata.lane_buf[i].signaltowatch<<std::endl;
			std::cout<<"\t (x,y) = ("<< newdata.lane_buf[i].our_x<<","<<newdata.lane_buf[i].our_y<<")"<<std::endl;
		}
		trafficsignal_buf.push_back(t);
		return 1;
	}


	void publishObjects(){
		// 현재에 each 지점에 red light인지 아닌지 판단을 도와주는 obstacle 만들기. 
		std::cout<<"------- STEP 3: PUBLISHING OBJECTS------"<<std::endl;

		geometry_msgs::PoseArray redlight_buf;
		redlight_buf.header.stamp = ros::Time::now();
		redlight_buf.header.frame_id = "map";

		// FOR EACH INTERSECTION (14개 정도)
		for (int i=0; i < trafficsignal_buf.size(); i++) {
			std::cout<<"-"<<std::endl;
			std::cout<<"intersection number: "<<trafficsignal_buf[i].id<<std::endl;
			auto& this_intersection = trafficsignal_buf[i];

			// CHECK STRUCTURE OF THIS INTERSECTION
			// print_structure(this_intersection);

			#if SOCKET
			// ASSERTION 1: only use SPAT data which are received within 20 seconds. 
			if (this_intersection.signal.timenow + 20 > ros::Time::now().toSec()){continue;}
			#endif 


			// //// WHICH LANE is LANE NUMBER 14 OR 15?
			// for (int a=0; a < this_intersection.intersection.lane_buf.size();a++){
			// 	if ((this_intersection.intersection.lane_buf[a].no  == 14)||(this_intersection.intersection.lane_buf[a].no  == 15)){
			// 		geometry_msgs::Pose redlight;
			// 		redlight.position.x = this_intersection.intersection.lane_buf[a].our_x;
			// 		redlight.position.y = this_intersection.intersection.lane_buf[a].our_y;
			// 		redlight_buf.poses.push_back(redlight);
			// 	}
			// }


			// FOR EACH NON-GREEN LIGHT SIGNAL ( red,yellow,or  almost red light )
			for (int j=0; j < this_intersection.signal.signalgroup_buf.size(); j++){
				// green light 표기가 아니라면...

				if (this_intersection.signal.signalgroup_buf[j].greenlight == false){
					for (int k=0; k< this_intersection.intersection.lane_buf.size(); k++){
						// signaltowatch
						
												// std::cout<<this_intersection.signal.signalgroup_buf[j].no<< " =?= ";
												// std::cout<<this_intersection.intersection.lane_buf[k].signaltowatch<<std::endl;

						if (this_intersection.signal.signalgroup_buf[j].no == this_intersection.intersection.lane_buf[k].signaltowatch ){
							geometry_msgs::Pose redlight;
							redlight.position.x = this_intersection.intersection.lane_buf[k].our_x;
							redlight.position.y = this_intersection.intersection.lane_buf[k].our_y;
							redlight_buf.poses.push_back(redlight);
						}
					}
				}
			}

		}

		object_marker.publish(redlight_buf);
	}

	void print_structure(struct TrafficSignal &intersection){
		std::cout<<"id: "<<intersection.id;
		std::cout<<" (.signal.id: "<<intersection.signal.id;
		std::cout<<" (.intersection.id: "<<intersection.intersection.id<<std::endl;

		std::cout<<"- MAP -"<<std::endl;
		for (int i=0; i<intersection.intersection.lane_buf.size();i++){
			std::cout<<"\tlane: "<< intersection.intersection.lane_buf[i].no<<std::endl;
			std::cout<<"\tsignal to watch: "<< intersection.intersection.lane_buf[i].signaltowatch<<std::endl;
			std::cout<<"\t (x,y) = ("<< intersection.intersection.lane_buf[i].our_x<<","<<intersection.intersection.lane_buf[i].our_y<<")"<<std::endl;
		}

		std::cout<<"- SPAT -"<<std::endl;
		for (int i=0; i<intersection.signal.signalgroup_buf.size();i++){
			std::cout<<"\tSignal Group: "<<intersection.signal.signalgroup_buf[i].no<<std::endl;
			std::cout<<"\tgreen light?: "<<(bool)intersection.signal.signalgroup_buf[i].greenlight<<std::endl;
			std::cout<<"\ttime left: "<<intersection.signal.signalgroup_buf[i].time_left<<std::endl;
		}
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
		start_time = ros::Time::now().toSec(); 		// TIME MEASURE START
		//// STEP 1
					//update messageFrame1 and messageFrame2 // this doesn't happen for
					#if ROSBAG
						ros::spinOnce();
					#endif
					#if SOCKET
						vc.do_recv();
					#endif
		mid_time = ros::Time::now().toSec(); 		// TIME MEASURE MID
		//// STEP 2
					int run_result = vc.run();
					if(run_result == -1){
						ROS_ERROR("msgFrame1 or msgFrame2 is empty, another recv() required!");
					} else if(run_result == 2){
						ROS_INFO("same MAP data received");
					} else if (run_result == 3){
						ROS_INFO("this intersection data already available");
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
