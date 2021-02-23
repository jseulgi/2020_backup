////////////////////////////////////////
// 	VERSION INFO
//	this version works for the page mill road.
//  -> next version: try to do this for Daegu Road
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
// LLtoUTM
#include "conversions.h"
#include "path_msgs/Map.h"
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
	ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher marker_pub = private_nh_.advertise<visualization_msgs::Marker>("marker", 0);
	ros::Publisher markerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("markerArray", 1);
	ros::Publisher textmarkerarray_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("textmarkerArray", 1);
	ros::Publisher traffic_marker_array_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("traffic_marker_array", 1);
	ros::Publisher traffic_text_marker_array_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("traffic_text_marker_array", 1);
	
	ros::Subscriber sub1 = private_nh_.subscribe<>("/map_info", 1, &V2X_Client::mapInfoCallback, this);
	
	// Class properties
	sockaddr_in obu_addr_;
	std::string obu_ip_;
	int obu_port_, sock_, buff_size_, size_, rate_, is_debug_on_;

	// For Map Translation
	double offset_x, offset_y;

	// Intersection vs Signal Group and its value: 2D array 
		// number of intersections (Daegu):  14개 id={1001,1002,1003, ... , 1014}
		// number of Signal Groups (표준): maximum 해봤자 6개? 보통은 4개일듯. 그래도 안전하게 10개라고 할까?  
			//	number of signal types: 0~9
	int traffic_signal[14][18][2]; //14 rows of intersections X 10 columns of signals groups X (signal , duration)

	double intersection_lane_xy[14][40][2];    //intersections X lanes의 node[0] X  (x,y)좌표

	// CHECK FLAG PARAMETER
	int is_connect_;

	std::string hex_spat_payload_str3 = "001380820018800001F80100004EB3593890B001043C2C192D5B2F7C001022E160C960C9675800C10F0B064B0D4B5340080878583259165C4A005043C2C192D5B2F7C003022E160C960F169B001C10F0B064B0D4B5340100878583259165C7200A043C2C192DAC3040006021E160C96459712803810F0B064B6B0C1000200878583259165C72";
	std::string hex_map_payload_str3 = "00128500280B3019000003F000A5F99BA5913E366E0832014A028C3894B0045000000A40041C6373983B916998761FB9D12EC0D9B495F3E0464AD53136148B40002247C80010B0085000000800041CAB323C3B6A68B876D1BEE12F39DCD095975DBE4ADDF122148BC00020412000B00C5000000400021CFCF0A83B8E6B1C55A17042B77BAC04872000396020A000000A0006BA6FDE5811E21DB635263ACFE08096DEE9C618EFCC0920C2000E90F40007081C000C029100000100004BBDB5630121E25AB9A1416483A6D98B25953926300C4400000400010ED1566B25A019F51647626BECB255F3840580E6800000520010F228FED1F2BCF264E6C4F0525DEAAEB8A4148001923F0000C58106800000400010F122ED31F3B4EB44E2F8F7F25E96AB50248200018B024D000000800021DF95BBC3E671DC4B380ADA083E239E7852430000C10A8002C0A340000010000C5E3C9563E6B9DA89A0124762C605851630B3F0A4590000421700058166800000280012BA44E2811E21F4B4E8A4C90935F07EF110A43C4000522E800023818840000040000AE5F021E048785A78E5C3D4130BC021B000381A8400000400010E8C875F25CA4B0E13117533C9ACDA43A110F800800001F88E0721000001000043A81A35497732C2E4C21549626AF692F84442002000007E2381E8400000400010EB249EB25DFCB029308B52CC9AB1A4BC3111800800001F8844200070410800000100012D28DB2518B7B592497F9AAF401091002C11540000008000D716E0FD0243C344422FC8B13A5A243D8487E21BC24470243080020B0495000000800031B85CBDC33BB8E28691028F90FCF21F086D90F6E0905800082C13540000020000C6CDB3620CF6A3BA1A2A0A0C4412C8A321B503EB0A41A000202290005828A800000200022D7987BD0478867BF1E50D10A4E822AE6830092C0C202D40280A4810000C22B000582AA800000280018D5A283D19F30884344012DC8B3223EE1804688522E20006921400031816620000028001167B151E8243C33F112A86838292916E94AD068AF29B10F611AE4705D8800000800051A3A92EA3401915067FE26D0CDD43251A85CDFE3599A364896C9874011880018186200000140005AB3D8998E08F105E00EC21EBE3AA960CBA00000048002329B426088F7D1DC33325CA4122C400045834E800000400018CBFB16C223B94710CB616612104D7C610F41AD54521B000041198002C1B740000020000C66809321128BA3906544B3110839BE1C87BF5712090E800022C1C740000020000C67689AF11202A5146593B1610936BB6C8736D8A8290F8000208EC00160EBA000000800083419524C629FC600CF135172201D52408C1ACC8458B38C29054000508F400160F3A000000A0006B47555FC11E218E7F03C32445CF063FCBCC04ADC41148320002A48440014707E0800000800035AEDB222090F11105A6AC8699D87A4429E9C821EB75528441A002000007DE38410400000400018D5EB7F1222694D990CA1B0C488B052C843CBEAB50883C00400000FBC70860800000800031A8D2DE44453E9B42166766A0C5A587221E2157C0C422002000007DE1228001C228200000040004AAD6D3AF411EF328866257A8011C100042360008300008DCD1BB51DA915D82C1D56F149210007249900038109180020D00043CA84598488A3D6242015D14451EE0A48D0004124A40020084AC0010680021C8FC1142776C241A2B295C2374A9D149220005249880028109980020C000232463BFC758DB728B13F7445246400189254000C0";

public:
	V2X_Client(void):
		private_nh_("~")
	{
		initParams();
		initSetup();
		ROS_INFO("V2X CLIENT NODE INITIALIZED");
		
		
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
		MessageFrame_t* msgFrame1 = nullptr;
		auto res1 = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame1, array_spat , sizeof(array_spat), 0,0);
		
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
		MessageFrame_t* msgFrame2 = nullptr;
		auto res2 = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame2, array_map , sizeof(array_map), 0,0);


		


		while (!is_done_) {
			if (msgFrame1->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
				fprintf(stderr,"\n\n\n[recv spat]\n");
				auto &spattmp = msgFrame1->value.choice.SPAT; 
				// asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame1);

				for (int i = 0; auto& iti = spattmp.intersections.list.array[i]; ++i) {
					std::cout<<"-----\nintersection index " << i << std::endl;
					std::cout<<"intersection id is : "<<(int)iti->id.id<<std::endl;

					for (int j = 0; auto& itj = iti->states.list.array[j]; j++){
						std::cout << "-" << std::endl;
						// if (itj->signalGroup){std::cout<<"signal group : "<<itj->signalGroup<<std::endl;}
						auto& itk = itj->state_time_speed.list.array[0];
						
						traffic_signal[i][itj->signalGroup][0] = itk->eventState;
						traffic_signal[i][itj->signalGroup][1] = (double)itk->timing->minEndTime/10;

						std::cout<< "signal group "<<itj->signalGroup<<"\'s event state is : " << traffic_signal[i][itj->signalGroup][0] << std::endl;
						std::cout<<"signal (probably) ends in "<< traffic_signal[i][itj->signalGroup][1] <<" seconds"<<std::endl;

						// std::cout<< "event state is : " << itk->eventState <<", which means ";
						// if 		(itk->eventState == 0){std::cout << "unavailable" << std::endl;}
						// else if (itk->eventState == 1){std::cout << "dark (unlit)" << std::endl;}
						// else if (itk->eventState == 2){std::cout << "stop then proceed (flashing red)" << std::endl;}
						// else if (itk->eventState == 3){std::cout << "stop and remain (red)" << std::endl;}
						// else if (itk->eventState == 4){std::cout << "pre-movement (red+yellow, get ready for green)" << std::endl;}
						// else if (itk->eventState == 5){std::cout << "permissive movement allowed (proceed with caution)" << std::endl;}
						// else if (itk->eventState == 6){std::cout << "protected movement allowed (proceed)" << std::endl;}
						// else if (itk->eventState == 7){std::cout << "permissive clearance (yellow, prepare to stop)" << std::endl;}
						// else if (itk->eventState == 8){std::cout << "protected yellow (...?)" << std::endl;}
						// else if (itk->eventState == 9){std::cout << "caution conflicting traffic (flashing yellow, proceed with caution)" << std::endl;}
 
					}
				}
				// ROS_INFO_STREAM("traffic signal matrix is ...");
				// ROS_INFO_STREAM(traffic_signal); //나중에 eigen으로 하면 표로 나타낼수있음.
			}  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(msgFrame2->messageId == dsrc_msg_id::MAP_DATA){
				fprintf(stderr,"\n\n\n[recv map]\n");
				auto &maptmp = msgFrame2->value.choice.MapData; 
				
				// asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame2);
				if(maptmp.layerType){
					std::cout<<"map type is "<<*maptmp.layerType;
					std::cout<<"! 3 means this is an intersectionData"<<std::endl;
				} 
				
				static auto map_count = 0;
				if(maptmp.msgIssueRevision){
					// if (map_count == maptmp.msgIssueRevision){	continue;	} //prevents parsing of additional map data
					map_count = maptmp.msgIssueRevision;
					std::cout<<"maptmp.msgIssueRevision is "<< map_count <<std::endl;
				} 
				// if(maptmp.regional){
				// 	std::cout<<"maptmp.regional not null"<<std::endl;
				// } 
				// if(maptmp.restrictionList){
				// 	std::cout<<"maptmp.restrictionList not null"<<std::endl;
				// } 
				// if(maptmp.roadSegments){
				// 	std::cout<<"maptmp.roadSegments not null"<<std::endl;
				// } 
				// if(maptmp.timeStamp){
				// 	std::cout<<"maptmp.timeStamp not null"<<std::endl;
				// } 

				for (int i=0; auto& iti = maptmp.intersections->list.array[i]; i++){
					if(iti->id.id){std::cout<<"intersection id is "<<iti->id.id<<std::endl;}
					std::cout<<"intersection ref position is (lat,long,elev) = ("<<iti->refPoint.lat<<","<<iti->refPoint.Long<<","<<*iti->refPoint.elevation<<")"<<std::endl;
					
					double northing, easting,t_northing, t_easting;
					std::string zone;

					//// MARKERS FOR THE LANE NODES
					visualization_msgs::MarkerArray marker_array;
					visualization_msgs::MarkerArray text_marker_array;
					visualization_msgs::Marker text_marker;
					
					visualization_msgs::Marker marker;
						marker.header.frame_id = "my_frame";
						marker.header.stamp = ros::Time::now();
						marker.ns = "reference point";
						marker.id = 0;
						marker.type = visualization_msgs::Marker::SPHERE; 
						marker.action = visualization_msgs::Marker::ADD;

						t_northing = (iti->refPoint.lat);
						t_easting = (iti->refPoint.Long);
						t_northing = t_northing/10000000;
						t_easting = t_easting/10000000;
						gps_common::LLtoUTM(t_northing,t_easting, northing, easting, zone);
						ROS_INFO("northing is : %f",northing);
						ROS_INFO("easting is : %f",easting);
						std::cout<<"zone is : "<<zone<<std::endl;

						marker.pose.position.x = northing;//iti->refPoint.Long;
						marker.pose.position.y = easting;//iti->refPoint.lat;
						marker.pose.position.z = 0;
						marker.pose.orientation.x = 0;
						marker.pose.orientation.y = 0;
						marker.pose.orientation.z = 0;
						marker.pose.orientation.w = 1;
						marker.lifetime = ros::Duration();
						marker.scale.x = 0.5;
						marker.scale.y = 0.5;
						marker.scale.z = 0.5;
						marker.color.r = 1.0;
						marker.color.g = 0.0;
						marker.color.b = 0.0;
						marker.color.a = 1.0;
					marker_pub.publish(marker);
					std::cout<<"marker published"<<std::endl;


					//// MARKERS FOR THE TRAFFIC SIGN
					visualization_msgs::MarkerArray traffic_marker_array;
					visualization_msgs::MarkerArray traffic_text_marker_array;					


					
					for (int j=0; auto& itj = iti->laneSet.list.array[j]; j++){
						//////
									// if(itj->laneID > 2){continue;}
						//////
						// if(itj->laneID){std::cout<<"\n-\nlane id is : " << itj->laneID << std::endl;}

						// if(itj->ingressApproach){std::cout<<"lane ingress Approach is : "<<*itj->ingressApproach<<std::endl;} //STOP CARING ABOUT THEM
						// if(itj->egressApproach){std::cout<<"lane egress Approach is : "<<*itj->egressApproach<<std::endl;}

						for (int k=0; auto& itk = itj->nodeList.choice.nodes.list.array[k]; k++){
							// std::cout<<"\tnode XY number: "<<k<<std::endl;
							// std::cout<<"\t\tLAT : "<<itk->delta.choice.node_LatLon.lat<<", LONG: "<<itk->delta.choice.node_LatLon.lon<<std::endl;
							
							double tt_northing, tt_easting;
							
							tt_northing = ((double)iti->refPoint.lat  + (double)itk->delta.choice.node_LatLon.lat)/10000000;
							tt_easting = ((double)iti->refPoint.Long + (double)itk->delta.choice.node_LatLon.lon)/10000000;
							gps_common::LLtoUTM(tt_northing,tt_easting, northing, easting, zone);

							marker.ns = "lane" + std::to_string(itj->laneID);
							marker.id = k;
							if (k == 0){ // first node position that will be used to situate markers 
								intersection_lane_xy[i][itj->laneID][0] = northing;
								intersection_lane_xy[i][itj->laneID][1] = easting;
							}
							marker.pose.position.x = northing;
							marker.pose.position.y = easting;

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
					for (int j=0; auto& itj = iti->laneSet.list.array[j]; j++){
						if(itj->ingressApproach){ //교차로 기준 들어오는 LANE일 때만 신호등 signal group가 표시되어있으니까
							std::cout<<"\n-\nlane id is : " << itj->laneID << std::endl;
							std::cout<<"this lane is an ingress Approach of : "<<*itj->ingressApproach<<std::endl;
							// std::cout<<"this lane is an egress Approach of : "<<*itj->egressApproach<<std::endl;
							
							// Traffic Sign with LINE_STRIP //
							visualization_msgs::Marker traffic_marker;
								traffic_marker.header.frame_id = "my_frame";
								traffic_marker.header.stamp = ros::Time::now();
								traffic_marker.ns = "temp_name"; //temp name
								traffic_marker.id = 1000; //temp number
								traffic_marker.type = visualization_msgs::Marker::LINE_LIST; 
								traffic_marker.action = visualization_msgs::Marker::ADD;
								geometry_msgs::Point lane1;
									lane1.x = intersection_lane_xy[i][itj->laneID][0];
									lane1.y = intersection_lane_xy[i][itj->laneID][1];
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
								for (int k=0; auto& itk = itj->connectsTo->list.array[k]; k++){
									if(itk->connectingLane.lane){
										std::cout<<"\titk->connectingLane.lane is : "<<itk->connectingLane.lane<<std::endl;
										traffic_marker.ns = "from lane " + std::to_string(itj->laneID) + " to lane " + std::to_string(itk->connectingLane.lane);
										traffic_marker.id = k;
										
										traffic_marker.points.push_back(lane1);
										geometry_msgs::Point lane2;
											lane2.x = intersection_lane_xy[i][itk->connectingLane.lane][0];
											lane2.y = intersection_lane_xy[i][itk->connectingLane.lane][1];
											lane2.z = 0;						
										traffic_marker.points.push_back(lane2);
									}

									if(itk->signalGroup){

										std::cout<<"\t\tSignal group to check is : " << *(itk->signalGroup) <<std::endl;
										//	참고자료
											// traffic_signal[14][18][2]; //14 rows of intersections X 10 columns of signals groups X (signal , duration)
										if 			(traffic_signal[i][*itk->signalGroup][0] == 3){			// RED 
											traffic_marker.color.r = 1.0; 
											traffic_marker.color.g = 0.0; 
											traffic_marker.color.b = 0.0;
										} else if 	(traffic_signal[i][*itk->signalGroup][0] == 5){			// GREEN
											traffic_marker.color.r = 0.0; 
											traffic_marker.color.g = 1.0; 
											traffic_marker.color.b = 0.0;										
										} else if	(traffic_signal[i][*itk->signalGroup][0] == 7){			// STOPPING YELLOW
											traffic_marker.color.r = 0.0; 
											traffic_marker.color.g = 1.0; 
											traffic_marker.color.b = 1.0;
										} else if 	(traffic_signal[i][*itk->signalGroup][0] == 9){			// BLINKING YELLOW
											traffic_marker.color.r = 0.0; 
											traffic_marker.color.g = 1.0; 
											traffic_marker.color.b = 1.0;
											// how to make it blink?
										} else 		{
											std::cout<<"other SIGNAL CASES"<<std::endl;
											// Do I need to make an counting thing that checks how many other cases are existant?
										}
									}

									traffic_marker_array.markers.push_back(traffic_marker);
									traffic_marker.points.clear();
									// traffic_text_marker_array.markers.push_back(text_traffic_marker);
								}					
							}
						}	
					}
					
					markerarray_pub.publish(marker_array);
					textmarkerarray_pub.publish(text_marker_array);
					marker_array.markers.clear();
					text_marker_array.markers.clear();
					
					traffic_marker_array_pub.publish(traffic_marker_array);
					// traffic_text_marker_array_pub.publish(traffic_text_marker_array);
					traffic_marker_array.markers.clear();
					// traffic_text_marker_array.markers.clear();
					

				}
    			


			}
			
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// char raw_data[buff_size_];
			// std::string msgs;

			// if (((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) > 0)) {

			// 	ROS_INFO("CORRECT PACKET RECEIVED, [CURRENT: %d Bytes / MINIMUM: %d Bytes]", size_, (int)sizeof(obu_tcp_header_t));
			// 	msgs.append((char *)raw_data, sizeof(raw_data));

			// 	while (msgs.size() != 0) {
			// 		std::string payload;
			// 		obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];
			// 		payload.append(msgs, sizeof(obu_tcp_header_t), header->payload_size);

			// 		if (is_debug_on_){
			// 		ROS_INFO("[MESSAGE TYPE: 0x%x]", header->packet_type);
			// 		ROS_INFO("[SEQUENCE: %d]", header->current_sequence);
			// 		ROS_INFO("[PAYLOAD SIZE: %d]", header->payload_size);
			// 		ROS_INFO("[DEVICE TYPE: 0x%X]", header->device_type);
			// 		ROS_INFO("[DEVICE ID: %02X-%02X-%02X]", header->device_id[2], header->device_id[1], header->device_id[0]);
			// 		ROS_INFO("[PAYLOAD: 0x%s]", convertArrToHexStr((char*)payload.c_str(), payload.size()).c_str());
			// 		}

			// 		// DECODING PAYLOAD
			// 		MessageFrame_t* msgFrame = nullptr;
			// 		auto res = uper_decode(0, &asn_DEF_MessageFrame, (void**)&msgFrame, payload.c_str(), payload.size(), 0, 0);

			// 		if (res.code ==asn_dec_rval_code_e::RC_OK ) {
			// 			ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");

			// 			// FOR ROS PUBLISHING //
			// 			std_msgs::String ros_msgs;
			// 			ros_msgs.data = msgs;
			// 			chatter_pub.publish(ros_msgs);
			// 			msgs.erase(0, sizeof(obu_tcp_header_t) + payload.size());
			// 			// FOR ROS PUBLISHING //

			// 			if  (msgFrame->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
			// 				ROS_INFO("SPAT MESSAGE RECIEVED");
			// 				//printing human readable stucture message to stdout
			// 				if (debug_on_){asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);}
							


			// 				break;
			// 			} else if (msgFrame->messageId == dsrc_msg_id::MAP_DATA){
			// 				ROS_INFO("MAP MESSAGE RECIEVED");
			// 				//printing human readable stucture message to stdout
			// 				if (debug_on_){asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);}							
							
			// 				break;
			// 			} else {
			// 				ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
			// 				break;
			// 			}
						
												
			// 			// std::cout << "- my parsing -" << std::endl;
			// 			// if(msgFrame->messageId == 19){
			// 				// std::cout << "msgFrame->messageId :"<< msgFrame->messageId <<"is SPAT"<<std::endl;
			// 				// std::cout << "msgFrame->value :" << std::endl; 
							
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[0]->signalGroup);
			// 				// std::cout << "ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.size);"<<std::endl;
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.size);
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list);
			// 				// std::cout << "msgFrame->value.choice :"<<std::endl; 
			// 				// ROS_INFO_STREAM( msgFrame->value.choice);
			// 				// std::cout << "msgFrame->value.choice.BasicSafetyMessage :"<<std::endl; 
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.BasicSafetyMessage);
			// 				// std::cout << "msgFrame->value.choice.SPAT :"<<std::endl; 
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT);
			// 				// std::cout << "msgFrame->value.choice.MapData :"<<std::endl; 
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.MapData);
			// 				// std::cout << "msgFrame->value.choice.SPAT.intersections.list.array[0] :" <<std::endl; 
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0] );
			// 			// } else if (msgFrame->messageId == 18){
			// 				// std::cout << "msgFrame->messageId :"<< msgFrame->messageId <<std::endl;
			// 				// std::cout << "msgFrame->value :" << std::endl; 
			// 				// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[0]->signalGroup);
			// 			// }


			// 			// std::cout<<msgFrame->IntersectionStateList.intersectionState.id.id<<std::endl;
			// 			// asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame);
			// 			break;
			// 		} else if (res.code == asn_dec_rval_code_e::RC_WMORE){
			// 			ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
			// 			break;
			// 		} else if (res.code == asn_dec_rval_code_e::RC_FAIL){
			// 			ROS_INFO("FAILED TO DECODE PAYLOAD");
			// 			break;						
			// 		} else{
			// 			break;
			// 		}
			// 	}
			// }
			//
			// else ROS_INFO("INCORRECT PACKET RECEIVED, [CURRENT: %d Bytes / MINIMUM: %d Bytes]", size_, (int)sizeof(obu_tcp_header_t));

			ROS_INFO("Now going to sleep for %f seconds",(float)rate_/1000000);

			usleep(rate_);
			
		}
		// close(sock_);
	}

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		private_nh_.getParam("debug_on", is_debug_on_);
		if (is_debug_on_){ROS_INFO_STREAM("debugging is on");}
		
		sock_, size_, is_connect_ = -1;
	}

	void initSetup(void)
	{
		initSignalHandler();
		memset(&obu_addr_, 0, sizeof(obu_addr_));
		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_port = htons(uint32_t(obu_port_));

		// if (inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr) <= 0) {
		// 	ROS_ERROR("FAILED TO CONVERT IP ADDRESS TO BINARY TYPE");
		// 	ros::shutdown();
		// }

		// if ((sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		// 	ROS_ERROR("FAILED TO CREATE SOCKET FILE DESCRIPTOR");
		// 	ros::shutdown();
		// }

		// if ((is_connect_ = connect(sock_, (const sockaddr *)&obu_addr_, sizeof obu_addr_)) < 0) {
		// 	ROS_ERROR("FAILED TO CONFIGURE TCP/IP SOCKET COMMUNICATION");
     	// ros::shutdown();
		// }
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

	void mapInfoCallback(const path_msgs::Map msg)
	{
		offset_x = msg.OffsetMapX;
		offset_y = msg.OffsetMapY;
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_client_node");

	V2X_Client vc;

	return 0;
}
