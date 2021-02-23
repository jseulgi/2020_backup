// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
// SIGNAL
#include <signal.h>
// SOCKET
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <unistd.h>

//// FOR TESTING ////
#include "obu_message.h"
// #include "sample_message.h"/
#include "sample_msg_01.h"


#include "MessageFrame.h"
#include "dsrc_msg_id.h"

// BSM PARAMETER
#define SIZE_ENCODE_BUF_MAX 2048
#define SIZE_BSM_WHEELBRAKES_BUF 1

bool is_done_ = false;

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class V2X_Client {
private:
	ros::NodeHandle private_nh_;
	ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("chatter", 1000);
	sockaddr_in obu_addr_;
	std::string obu_ip_;
	int obu_port_, sock_, buff_size_, size_, rate_, is_debug_on_, is_simul_on_;

	// CHECK FLAG PARAMETER
	int is_connect_;
public:
	V2X_Client(void):
		private_nh_("~")
	{
		initParams();
		initSetup();
		ROS_INFO("V2X CLIENT NODE INITIALIZED");

		while (!is_done_) {
			char raw_data[buff_size_];
			std::string msgs;

			if (((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) > 0)) {

				ROS_INFO("CORRECT PACKET RECEIVED, [CURRENT: %d Bytes / MINIMUM: %d Bytes]", size_, (int)sizeof(obu_tcp_header_t));
				msgs.append((char *)raw_data, sizeof(raw_data));

				while (msgs.size() != 0) {
					std::string payload;
					obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];
					payload.append(msgs, sizeof(obu_tcp_header_t), header->payload_size);

					if (is_debug_on_){
					ROS_INFO("[MESSAGE TYPE: 0x%x]", header->packet_type);
					ROS_INFO("[SEQUENCE: %d]", header->current_sequence);
					ROS_INFO("[PAYLOAD SIZE: %d]", header->payload_size);
					ROS_INFO("[DEVICE TYPE: 0x%X]", header->device_type);
					ROS_INFO("[DEVICE ID: %02X-%02X-%02X]", header->device_id[2], header->device_id[1], header->device_id[0]);
					ROS_INFO("[PAYLOAD: 0x%s]", convertArrToHexStr((char*)payload.c_str(), payload.size()).c_str());
					}

					// DECODING PAYLOAD
					MessageFrame_t* msgFrame = nullptr;
					auto res = uper_decode(0, &asn_DEF_MessageFrame, (void**)&msgFrame, payload.c_str(), payload.size(), 0, 0);

					if (res.code ==asn_dec_rval_code_e::RC_OK ) {
						ROS_INFO("DECODED PAYLOAD SUCCESSFULLY");

						// FOR ROS PUBLISHING //
						std_msgs::String ros_msgs;
						ros_msgs.data = msgs; /////////////////////////////////////////////////////////////////////////????????????? payload 가 맞지 않나?
						chatter_pub.publish(ros_msgs);
						msgs.erase(0, sizeof(obu_tcp_header_t) + payload.size());
						// FOR ROS PUBLISHING //

						if  (msgFrame->messageId == dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE){
							ROS_INFO("SPAT MESSAGE RECIEVED");
							//printing human readable stucture message to stdout
							if (is_debug_on_){asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);}
							


							break;
						} else if (msgFrame->messageId == dsrc_msg_id::MAP_DATA){
							ROS_INFO("MAP MESSAGE RECIEVED");
							//printing human readable stucture message to stdout
							if (is_debug_on_){asn_fprint(stdout, &asn_DEF_MessageFrame, msgFrame);}							
							
							break;
						} else {
							ROS_INFO("MESSAGE THAT I DON'T CARE RECEIVED");
							break;
						}
						
												
						// std::cout << "- my parsing -" << std::endl;
						// if(msgFrame->messageId == 19){
							// std::cout << "msgFrame->messageId :"<< msgFrame->messageId <<"is SPAT"<<std::endl;
							// std::cout << "msgFrame->value :" << std::endl; 
							
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[0]->signalGroup);
							// std::cout << "ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.size);"<<std::endl;
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.size);
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list);
							// std::cout << "msgFrame->value.choice :"<<std::endl; 
							// ROS_INFO_STREAM( msgFrame->value.choice);
							// std::cout << "msgFrame->value.choice.BasicSafetyMessage :"<<std::endl; 
							// ROS_INFO_STREAM(msgFrame->value.choice.BasicSafetyMessage);
							// std::cout << "msgFrame->value.choice.SPAT :"<<std::endl; 
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT);
							// std::cout << "msgFrame->value.choice.MapData :"<<std::endl; 
							// ROS_INFO_STREAM(msgFrame->value.choice.MapData);
							// std::cout << "msgFrame->value.choice.SPAT.intersections.list.array[0] :" <<std::endl; 
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0] );
						// } else if (msgFrame->messageId == 18){
							// std::cout << "msgFrame->messageId :"<< msgFrame->messageId <<std::endl;
							// std::cout << "msgFrame->value :" << std::endl; 
							// ROS_INFO_STREAM(msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[0]->signalGroup);
						// }


						// std::cout<<msgFrame->IntersectionStateList.intersectionState.id.id<<std::endl;
						// asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame);
						break;
					} else if (res.code == asn_dec_rval_code_e::RC_WMORE){
						ROS_INFO("MORE DATA REQUIRED, PLEASE CALL AGAIN");
						break;
					} else if (res.code == asn_dec_rval_code_e::RC_FAIL){
						ROS_INFO("FAILED TO DECODE PAYLOAD");
						break;						
					} else{
						break;
					}
				}
			}
			
			else ROS_INFO("INCORRECT PACKET RECEIVED, [CURRENT: %d Bytes / MINIMUM: %d Bytes]", size_, (int)sizeof(obu_tcp_header_t));

			ROS_INFO("Now going to sleep for %d seconds",rate_/1000000);
			usleep(rate_);
			
		}
		close(sock_);
	}

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		private_nh_.getParam("debug_on", is_debug_on_);
		private_nh_.getParam("simul_on", is_simul_on_);
		if (is_debug_on_){ROS_INFO_STREAM("debugging is on");}
		if (is_simul_on_){ROS_INFO_STREAM("debugging is on");}
		
		sock_, size_, is_connect_ = -1;
	}

	void initSetup(void)
	{
		initSignalHandler();
		memset(&obu_addr_, 0, sizeof(obu_addr_));
		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_port = htons(uint32_t(obu_port_));

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
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_client_node");

	V2X_Client vc;

	return 0;
}
