// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <fstream>
#include <string.h>
#include <cmath>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <fstream>
#include <bitset>
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

#include <stdexcept>

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class Call_List_Client {
public:	
	Call_List_Client(void):
		private_nh_("~")	
	{
		initParams();
		initSetup();

		ROS_INFO("CALL LIST CLIENT NODE INITIALIZED");
		
		nav_msgs::Odometry odom_buf_temp;	
			odom_buf_temp.pose.pose.position.x = 35.880143;
			odom_buf_temp.pose.pose.position.y = 128.626841;
			odom_buf_temp.pose.pose.position.z = 100;
		odom_buf_.push_back(odom_buf_temp);

		// while (odom_buf_.size() == 0) {
		// 	ROS_INFO("WAITING FOR INITIAL VEHICLE ODOMETRY MESSAGE");
		// 	ros::spinOnce();
		// }

		if (!requestConnection()) ros::shutdown();
		
		ros::Rate loop_rate(rate_);

		while (ros::ok()) {
			char raw_data[buff_size_];
			std::string msgs;
			if ((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) >= 9) {
				ROS_INFO("CORRECT PACKET RECEIVED, [CURRENT: %d bytes / MINIMUM: %d bytes]", (int)size_, (int)sizeof(obu_tcp_header_t));
				msgs.append((char *)raw_data, size_);
				checkResponse(msgs);	
			}

			sendCurrentLocation();

			ros::spinOnce();
			loop_rate.sleep();
		}			

		// if (&pay != NULL){
		// 	practice();
		// } else {

		// 	if (!requestConnection()) ros::shutdown();
			
		// 	ros::Rate loop_rate(rate_);

		// 	while (ros::ok()) {
		// 		char raw_data[buff_size_];
		// 		std::string msgs;
		// 		if ((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) >= 9) {
		// 			ROS_INFO("CORRECT PACKET RECEIVED, [CURRENT: %d bytes / MINIMUM: %d bytes]", (int)size_, (int)sizeof(obu_tcp_header_t));
		// 			msgs.append((char *)raw_data, size_);
		// 			checkResponse(msgs);	
		// 		}

		// 		sendCurrentLocation();

		// 		ros::spinOnce();
		// 		loop_rate.sleep();
		// 	}			

		// }


	}

	int practice(){
		// std::string pay_trial = string_to_hex(pay);
		std::string pay_trial = "1";
		ROS_INFO("NUMBER OF CALLS: %d", (int)pay_trial[0]);
		ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)pay_trial[1]);
		ROS_INFO("NUMBER OF IRREGULARS: %d", (int)pay_trial[2]);

		std::vector<std::string> calls;
		int callData_size = 40;

		for (int i=0; i<(int)pay_trial[0]; i++) {
			std::cout<<"-\nmission no."<<i<<" is"<<std::endl;
			std::string call = pay_trial.substr(3 + i * callData_size, callData_size);
			int id = (int)call[0];
			int m_status = (int)call[1]; //0x00 대기, 0x01 완료
			int m_types = (int)call[2]; // 0001: 좌회전, 0010: 터널, 0100: no HD map, 1000: 비정현
			std::bitset<8> m_bit = m_types; 
				if (m_bit[0] == true){ROS_INFO("\t- left turn mission");} 
				if (m_bit[1] == true){ROS_INFO("\t- tunnel	 mission");} 
				if (m_bit[2] == true){ROS_INFO("\t- no HD map mission");} 
				if (m_bit[3] == true){ROS_INFO("\t- obstacle	 mission");} 
			unsigned short m_score;
			unsigned short m_distance;
			double slat;
			double slong;
			double elat;
			double elong;
			memcpy(&m_score, &call[3], 2);
			memcpy(&m_distance, &call[5], 2);
			uint8_t m_irregualr_id = (int)call[7];
			memcpy(&slat, &call[8], 8);
			memcpy(&slong, &call[16], 8);
			memcpy(&elat, &call[24], 8);
			memcpy(&elong, &call[32], 8);

			std::cout<<"mission id is : "<<id<<std::endl;
			std::cout<<"mission status (0: 대기, 1:완료) is : "<<m_status<<std::endl;
			ROS_INFO("score is : %d",m_score);
			ROS_INFO("distance is : %d",m_distance);
			ROS_INFO("start lat is %f",slat);
			ROS_INFO("start long is %f",slong);
			ROS_INFO("end lat is %f",elat);
			ROS_INFO("end long is %f",elong);
			// std::cout<<"score is : "<<score<<std::endl;
			// std::cout<<"start lat is : "<<slat<<std::endl;
			// std::cout<<"start long is : "<<slong<<std::endl;
			// std::cout<<"end lat is : "<<elat<<std::endl;
			// std::cout<<"end long is : "<<elong<<std::endl;
		}		

		return 0;
	}

	//// 보험용!

	std::string string_to_hex(const std::string& input)
	{
		static const char hex_digits[] = "0123456789ABCDEF";

		std::string output;
		output.reserve(input.length() * 2);
		for (unsigned char c : input)
		{
			output.push_back(hex_digits[c >> 4]);
			output.push_back(hex_digits[c & 15]);
		}
		return output;
	}


	int hex_value(unsigned char hex_digit)
	{
		static const signed char hex_values[256] = {
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			0,  1,  2,  3,  4,  5,  6,  7,  8,  9, -1, -1, -1, -1, -1, -1,
			-1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		};
		int value = hex_values[hex_digit];
		if (value == -1) throw std::invalid_argument("invalid hex digit");
		return value;
	}

	std::string hex_to_string(const std::string& input)
	{
		const auto len = input.length();
		if (len & 1) throw std::invalid_argument("odd length");

		std::string output;
		output.reserve(len / 2);
		for (auto it = input.begin(); it != input.end(); )
		{
			int hi = hex_value(*it++);
			int lo = hex_value(*it++);
			output.push_back(hi << 4 | lo);
		}
		return output;
	}

	//// 보험용! END

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		sock_, size_, is_connect_ = -1;
		sequence_ = 0;
	}
	void initSetup(void)
	{
		odom_sub_ = nh_.subscribe("odom", 1, &Call_List_Client::odomCallback, this);
	}

	void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
		odom_buf_.emplace_back(*odom_msg);
		odom_buf_.back().header.stamp = ros::Time::now();
	}


	bool requestConnection(void) {
		memset(&obu_addr_, 0, sizeof(obu_addr_));
		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_port = htons(uint32_t(obu_port_));

		if (inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr) <= 0) {
			ROS_ERROR("FAILED TO CONVERT IP ADDRESS TO BINARY TYPE");
			return false;
		}

		if ((sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
			ROS_ERROR("FAILED TO CREATE SOCKET FILE DESCRIPTOR");
			return false;
		}

		if ((is_connect_ = connect(sock_, (const sockaddr *)&obu_addr_, sizeof obu_addr_)) == -1) {
			ROS_ERROR("FAILED TO CONFIGURE TCP/IP SOCKET COMMUNICATION");
			ROS_ERROR("connect(sock_, (const sockaddr *)&obu_addr_, sizeof obu_addr_)) is: %d",connect(sock_, (const sockaddr *)&obu_addr_, sizeof obu_addr_));
			// ROS_ERROR("obu_addr_ is %s",obu_addr_);
			return false;
		}		
	
		unsigned char permission_packet[31];

		double latitude = odom_buf_.back().pose.pose.position.x;
		// std::cout<<"latitude is : "<<latitude<<std::endl;
		double longitude = odom_buf_.back().pose.pose.position.y;
		// std::cout<<"longitude is : "<<longitude<<std::endl;
		float elevation = odom_buf_.back().pose.pose.position.z;
		// std::cout<<"elevation is : "<<elevation<<std::endl;
		
		memset(permission_packet, 0x00, sizeof(permission_packet));
		// permission_packet[0] = 0x31;
		// permission_packet[1] = 0x33;
		permission_packet[0] = 0x33;
		permission_packet[1] = 0x31;
		permission_packet[2] = sequence_;
		permission_packet[3] = 0x16;
		permission_packet[4] = 0x00;
		permission_packet[5] = 0xCE;
		permission_packet[6] = 0x3B;
		permission_packet[7] = 0x31;
		permission_packet[8] = 0x9E;
		unsigned char lat_arr[sizeof(latitude)];
		memcpy(lat_arr, &latitude, sizeof(latitude));
		unsigned char long_arr[sizeof(longitude)];
		memcpy(long_arr, &longitude, sizeof(longitude));
		unsigned char elev_arr[sizeof(elevation)];
		memcpy(elev_arr, &elevation, sizeof(elevation));
		unsigned char payload_arr[20];
		memcpy(payload_arr, lat_arr, sizeof(latitude));
		memcpy(&payload_arr[8], long_arr, sizeof(longitude));
		memcpy(&payload_arr[16], elev_arr, sizeof(elevation));
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 20);
		unsigned char crc_arr[2];
		memcpy(&permission_packet[9], lat_arr, sizeof(latitude));
		memcpy(&permission_packet[17], long_arr, sizeof(longitude));
		memcpy(&permission_packet[25], elev_arr, sizeof(elevation));
		memcpy(crc_arr, &check_result, 2);
		permission_packet[29] = crc_arr[0];
		permission_packet[30] = crc_arr[1];

		// std::cout<<std::hex;
		// std::cout<<(int)permission_packet[29]<<std::endl;
		// std::cout<<(int)permission_packet[30]<<std::endl;	
		
		int sentisize = -1;

		if (sentisize = send(sock_, permission_packet, sizeof(permission_packet), 0) > 0) {
			ROS_INFO("SEND ACCESS PERMISSION PACKET SUCCESSFULLY");
			sequence_++;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND PERMISSION PACKET, PLEASE CHECK CONNECTION STATUS");
			return false;
		}
	}
	std::string static convertArrToHexStr(char *data, int len) {
		std::string s(len * 2, ' ');
		for (int i=0;i<len;++i) {
			s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
			s[2 * i + 1] = hexmap[data[i] & 0x0F];
		}
		return s;
	}
	void checkResponse(std::string packet) {
		while (packet.size() != 0) {

				// std::cout<<"\npacket size received is: "<< (int)packet.size() <<std::endl;
				// std::cout<<"original total_packet size is: "<< (int)total_packet.size() <<std::endl;
			total_packet = total_packet + packet;
				// std::cout<<"new total_packet size is: "<< (int)total_packet.size() <<std::endl;
			std::string payload;
			
			obu_tcp_header_t *header = (obu_tcp_header_t *)&total_packet[0];
			unsigned short packet_type = header->packet_type;
			payload.append(total_packet, sizeof(obu_tcp_header_t), header->payload_size);
				// std::cout<<"sizeof(obu_tcp_header_t) is : " << sizeof(obu_tcp_header_t) <<std::endl;
				// std::cout<<"sizeof(header->payload_size) is : " << header->payload_size <<std::endl;			

			if (packet_type == 0x133F) {
				unsigned char error_code = payload[0];
				ROS_ERROR("[PACKET TYPE: 0x%2x] ACCESS RESTRICTION NOTIFICATION MESSAGE RECIEVED", packet_type);
				if (error_code == 0x01) ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED DEVICE ID, PLEASE CHECK DEVICE ID AGAIN", error_code);
				else if (error_code == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] LTE DEVICE NOT OFFERED FOR COMPETITION WAS USED, PLEASE CHECK LTE DEVICE AGAIN", error_code);
				else if (error_code == 0x03) ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED PACKET TYPE, PLEASE CHECK PACKET TYPE AGAIN", error_code);
				else if (error_code == 0x04) ROS_ERROR("[ERROR CODE: 0x%x] PACKET TYPE NOT IN SERVICE SCENARIO ORDER WAS USED, PLEASE CHECK PACKET TYPE AGAIN", error_code);
				else if (error_code == 0x05) ROS_ERROR("[ERROR CODE: 0x%x] INCORRECT PAYLOAD LEGNTH, PLEASE CHECK PAYLOAD LENGTH AGAIN", error_code);
				else if (error_code == 0x06) ROS_ERROR("[ERROR CODE: 0x%x] INCORRECT DEVICE TYPE, PLEASE REPLACE DEVICE TYPE TO 0xCE", error_code);
				else if (error_code == 0x07) ROS_ERROR("[ERROR CODE: 0x%x] SAME PACKET ALREADY SENT MORE THAN 3 TIMES", error_code);
				else if (error_code == 0x08) ROS_ERROR("[ERROR CODE: 0x%x] TCP CONNECTION ALREADY ESTABLISHED BY THE SAME IP", error_code);
				else if (error_code == 0x09) ROS_ERROR("[ERROR CODE: 0x%x] NO ACCESS PERMISSION PACKET IS SENT AFTER ESTABLISHING TCP CONNECTION", error_code);
				else if (error_code == 0x0A) ROS_ERROR("[ERROR CODE: 0x%x] DEVICE ID OF ANOTHER TEAM WAS USED, PLEASE CHECK DEVICE ID AGAIN", error_code);
				else if (error_code == 0x0B) ROS_ERROR("[ERROR CODE: 0x%x] CHECKSUM VALIDATION ERROR, PLEASE CHECK CRC16 AGAIN", error_code);
				else if (error_code == 0x0C) ROS_ERROR("[ERROR CODE: 0x%x] DISCONNECTION MORE THAN 10 TIMES DUE TO THE SAME ERROR, PLEASE REQUEST RELEASE TO HOST", error_code);
				else ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED ERROR CODE, MAYBE CAUSED BY PARSING ERROR", error_code);
				total_packet.clear();
				ros::shutdown();
			}
			else if (packet_type == 0x1334) {
				unsigned char response = payload[0];
				ROS_INFO("[PACKET TYPE: 0x%2x] ACCESS PERMISSION RESPONSE MESSAGE RECIEVED", packet_type);
				if (response == 0x01) ROS_INFO("ACCESS PERMISSION REQUEST APPROVED");		
				else if (response == 0x02) {
					unsigned char error_code = payload[1];
					ROS_ERROR("ACCESS PERMISSION REQUEST DENIED");
					if (error_code == 0x00) ROS_ERROR("[ERROR CODE: 0x%x] NO ERROR EXISTS", error_code);
					else if (error_code == 0x01) ROS_ERROR("[ERROR CODE: 0x%x] LATITUDE OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] LONGITUDE OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x03) ROS_ERROR("[ERROR CODE: 0x%x] ELEVATION OF VEHICLE IS HIGHER THAN RSU", error_code);
					else if (error_code == 0x04) ROS_ERROR("[ERROR CODE: 0x%x] ACCESS PERMISSION REQUEST ALREADY APPROVED BY SERVER", error_code);
					else if (error_code == 0x05) ROS_ERROR("[ERROR CODE: 0x%x] REPEATEDLY SEND ACCESS PERMISSION REQUEST BEFORE RECIEVING RESPONSE FROM SERVER", error_code);
					else ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED ERROR CODE, MAYBE CAUSED BY PARSING ERROR", error_code);
				}
				total_packet.clear();
			}
			else if (packet_type == 0x1335) {
				ROS_INFO("[PACKET TYPE: 0x%2x] LOCATION ERROR NOTIFICATION MESSAGE RECIEVED", packet_type);
				total_packet.clear();
			}
			//////////////////////////////// MISSION PART (START) //////////////////////////////////////////
			////////////																		////////////
			else if (packet_type == 0x1136) {
				if ( total_packet.size() < header->payload_size + 9){
					ROS_INFO("NEED %d BYTES! BUT ONLY GOT %d BYTES", (int)header->payload_size + 9,(int)total_packet.size());
					packet.erase(0, sizeof(obu_tcp_header_t) + payload.size());
					break;
				} else if ( total_packet.size()  == header->payload_size + 9){
					ROS_INFO("GOT ALL %d/%d BYTES REQUIRED!",(int)total_packet.size(),(int)header->payload_size + 9);
				} 
				std::cout<<"---"<<std::endl;
				ROS_INFO("[PACKET TYPE: 0x%2x] CALL LIST MESSAGE RECIEVED", packet_type);


				// 보험용
				std::string output = string_to_hex(payload); //https://stackoverflow.com/questions/3381614/c-convert-string-to-hexadecimal-and-vice-versa
					std::cout<<output<<std::endl;
					std::ofstream myfile;
					myfile.open("/home/suminee/mission_payload.txt");
					myfile << output;
					myfile.close();
				//

				/// PARSING PAYLOAD ///
				
				ROS_INFO("NUMBER OF CALLS: %d", (int)payload[0]);
				ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)payload[1]);
				ROS_INFO("NUMBER OF IRREGULARS: %d", (int)payload[2]);
				ROS_INFO("PAYLOAD SIZE IS : %d", (int)payload.size());

				std::vector<std::string> calls;
				int callData_size = 40;

				for (int i=0; i<(int)payload[0]; i++) {
					std::cout<<"-\nmission no."<<i<<" is"<<std::endl;
					std::string call = payload.substr(3 + i * callData_size, callData_size);
					int id = (int)call[0];
					int m_status = (int)call[1]; //0x00 대기, 0x01 완료
					int m_types = (int)call[2]; // 0001: 좌회전, 0010: 터널, 0100: no HD map, 1000: 비정현
					std::bitset<8> m_bit = m_types; 
						if (m_bit[0] == true){ROS_INFO("\t- left turn mission");} 
						if (m_bit[1] == true){ROS_INFO("\t- tunnel	 mission");} 
						if (m_bit[2] == true){ROS_INFO("\t- no HD map mission");} 
						if (m_bit[3] == true){ROS_INFO("\t- obstacle	 mission");} 
					unsigned short m_score;
					unsigned short m_distance;
					double slat;
					double slong;
					double elat;
					double elong;
					memcpy(&m_score, &call[3], 2);
					memcpy(&m_distance, &call[5], 2);
					uint8_t m_irregualr_id = (int)call[7];
					memcpy(&slat, &call[8], 8);
					memcpy(&slong, &call[16], 8);
					memcpy(&elat, &call[24], 8);
					memcpy(&elong, &call[32], 8);

					std::cout<<"mission id is : "<<id<<std::endl;
					std::cout<<"mission status (0: 대기, 1:완료) is : "<<m_status<<std::endl;
					ROS_INFO("score is : %d",m_score);
					ROS_INFO("distance is : %d",m_distance);
					ROS_INFO("start lat is %f",slat);
					ROS_INFO("start long is %f",slong);
					ROS_INFO("end lat is %f",elat);
					ROS_INFO("end long is %f",elong);

				}
				
				for (int i=0; i < 2; i++){
					std::string irregular_call = payload.substr(3+60*callData_size + i*65,65);
					int id = (int)irregular_call[0];
					std::cout<<"\nirregular object id is : "<<id<<std::endl;

					double lat1, long1, lat2, long2, lat3, long3, lat4, long4;
					memcpy(&lat1,&irregular_call[1],8);
					memcpy(&long1,&irregular_call[9],8);
					memcpy(&lat2,&irregular_call[17],8);
					memcpy(&long2,&irregular_call[25],8);
					memcpy(&lat3,&irregular_call[33],8);
					memcpy(&long3,&irregular_call[41],8);
					memcpy(&lat4,&irregular_call[49],8);
					memcpy(&long4,&irregular_call[57],8);

					std::cout<<"irregular object's coordinate is ..."<<std::endl;
					ROS_INFO("latitude  1 is : %f",lat1);
					ROS_INFO("longitude 1 is : %f",long1);
					ROS_INFO("latitude  2 is : %f",lat2);
					ROS_INFO("longitude 2 is : %f",long2);
					ROS_INFO("latitude  3 is : %f",lat3);
					ROS_INFO("longitude 3 is : %f",long3);
					ROS_INFO("latitude  4 is : %f",lat4);
					ROS_INFO("longitude 4 is : %f",long4);
				}



				


				// PARSING ENDS


				// CHOOSE WHICH MISSION TO PROCEED
					// MISSION PLANNING ALGORITHM
						// FOR NOW... CHOOSE THE CLOSEST STARTING POINT
					// int mission_id = 60;
				

				//// CALL REQUEST MESSAGE (NOTIFY CHOSEN MISSION) ////
				// CallRequestMessage(mission_id);
				

				// while(notified == false){
				// 	sleep(3);
				// }
					
				

				total_packet.clear();
			}
			////////////																		////////////
			//////////////////////////////// MISSION PART (END) ////////////////////////////////////////////
			else if (packet_type == 0x1338){ // 프로토콜 2.3.2.3 콜 응답(콜 매칭) 메시지
				if (total_packet.size() != 0 ){
					unsigned char matchingCalld = payload[0];
					unsigned char response = payload[1]; 
					unsigned char error_code = payload[2];
					ROS_INFO("[PACKET TYPE: 0x%2x] Call Response-Call Matching Message RECIEVED", packet_type);
					ROS_INFO("the requested mission id is %d",(int)matchingCalld);
					if (response == 0x00) ROS_INFO("MISSION REQUEST APPROVED");
					else {
						ROS_ERROR("MISSION REQUEST DENIED");
						if (response == 0x01) ROS_ERROR("[ERROR CODE: 0x%x] penalty-time(미션 포기) 중 요청한 경우",response);
						else if (response == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] 대회가 사정에 의해 일시 중지된 상태에서 요청한 경우", response);
						else if (response == 0x03) ROS_ERROR("[ERROR CODE: 0x%x] 미션 매칭이 성공 후, 미션이 완료되지 않은 상태에서 다른 미션을 요청한 경우", response);
						else if (response == 0x04) ROS_ERROR("[ERROR CODE: 0x%x] 이미 완료(성공/실패/포기)한 미션을 다시 요청한 경우", response);
						else if (response == 0x05) ROS_ERROR("[ERROR CODE: 0x%x] 이미 미션이 매칭되어 수행되는 중에 수행중인 동일한 미션을 요청한 경우 (차량이 승객의 승차지점으로 이동하기 전까지는 동일한 미션에 대한 요청은 에러코드로 판단하지 않음)", response);
						else if (response == 0x06) ROS_ERROR("[ERROR CODE: 0x%x] 요청한 콜을 이미 다른 팀에서 선점함 경우", response);
						else if (response == 0x10) {
							ROS_ERROR("[ERROR CODE: 0x%x] 콜 요청 프로토콜 오류에 의한 실패 (errorCode 0x11 또는 0x12에 해당하는 경우)", response);
							if (error_code == 0x11) ROS_ERROR("[ERROR CODE: 0x%x] V2X 서버가 콜 리스트를 송출하기 전에 요청을 한 경우", error_code);
							else if (error_code == 0x12) ROS_ERROR("[ERROR CODE: 0x%x] requestCallID의 값이 0x00 이거나 0x3C를 초과하는 경우", error_code);
						}
					}					
				}	
			}else {
				ROS_INFO("[PACKET TYPE: 0x%2x] UNDEFINED MESSAGE RECIEVED", packet_type);
			}
			total_packet.clear();
			packet.erase(0, sizeof(obu_tcp_header_t) + payload.size());
		}
	}
	bool sendCurrentLocation(void) {
		unsigned char location_packet[34];

		double latitude = odom_buf_.back().pose.pose.position.x;
		double longitude = odom_buf_.back().pose.pose.position.y;
		float elevation = odom_buf_.back().pose.pose.position.z;
		unsigned short heading = 0;
		unsigned char speed = 0;
		
		memset(location_packet, 0x00, sizeof(location_packet));
		location_packet[0] = 0x35;
		location_packet[1] = 0x31;
		// location_packet[0] = 0x31;
		// location_packet[1] = 0x35;
		location_packet[2] = sequence_;
		location_packet[3] = 0x19;
		location_packet[4] = 0x00;
		location_packet[5] = 0xCE;
		location_packet[6] = 0x3B;
		location_packet[7] = 0x31;
		location_packet[8] = 0x9E;
		unsigned char lat_arr[sizeof(latitude)];
		memcpy(lat_arr, &latitude, sizeof(latitude));
		unsigned char long_arr[sizeof(longitude)];
		memcpy(long_arr, &longitude, sizeof(longitude));
		unsigned char elev_arr[sizeof(elevation)];
		memcpy(elev_arr, &elevation, sizeof(elevation));
		unsigned char heading_arr[sizeof(heading)];
		memcpy(heading_arr, &heading, sizeof(heading));
		unsigned char speed_arr[sizeof(speed)];
		memcpy(speed_arr, &speed, sizeof(speed));
		unsigned char payload_arr[23];
		memcpy(payload_arr, lat_arr, sizeof(latitude));
		memcpy(&payload_arr[8], long_arr, sizeof(longitude));
		memcpy(&payload_arr[16], elev_arr, sizeof(elevation));
		memcpy(&payload_arr[20], heading_arr, sizeof(heading));
		memcpy(&payload_arr[22], speed_arr, sizeof(speed));
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 23);
		unsigned char crc_arr[2];
		memcpy(&location_packet[9], lat_arr, sizeof(latitude));
		memcpy(&location_packet[17], long_arr, sizeof(longitude));
		memcpy(&location_packet[25], elev_arr, sizeof(elevation));
		memcpy(&location_packet[29], heading_arr, sizeof(heading));
		memcpy(&location_packet[31], speed_arr, sizeof(speed));
		memcpy(crc_arr, &check_result, 2);
		location_packet[32] = crc_arr[0];
		location_packet[33] = crc_arr[1];

		// std::cout<<std::hex;
		// std::cout<<(int)location_packet[29]<<std::endl;
		// std::cout<<(int)location_packet[30]<<std::endl;	
		
		int sentisize = -1;

		if (sentisize = send(sock_, location_packet, sizeof(location_packet), 0) > 0) {
			ROS_INFO("SEND ACCESS LOCATION PACKET SUCCESSFULLY");
			sequence_++;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND LOCATION PACKET, PLEASE CHECK CONNECTION STATUS");
			return false;
		}
	}

	int CallRequestMessage(int mission_id){ //선점하고자하는 메시지를 보내는 것, 프로토콜 2.3.2.2임
		unsigned char call_packet[12];

		memset(call_packet, 0x00, sizeof(call_packet));
		call_packet[0] = 0x37;
		call_packet[1] = 0x31;
		call_packet[2] = sequence_++;
		call_packet[3] = 0x03;
		call_packet[4] = 0x00;
		call_packet[5] = 0xCE;
		call_packet[6] = 0x3B; 
		call_packet[7] = 0x31;
		call_packet[8] = 0x9E;
		call_packet[9] = (unsigned char) mission_id;
		unsigned char payload_arr[1];
		memset(payload_arr,0x00,sizeof(payload_arr));
		payload_arr[0] = (unsigned char) mission_id;
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 1);
		unsigned char crc_arr[2];
		memcpy(crc_arr, &check_result,2);
		call_packet[10] = crc_arr[0];
		call_packet[11] = crc_arr[1];
		
		int sentisize = -1;
		if (sentisize = send(sock_, call_packet, sizeof(call_packet), 0) > 0) {
			ROS_INFO("SEND CALL REQUEST PACKET SUCCESSFULLY");
			sequence_++;

			// char raw_data[20];
			// std::cout<<"checking for call request response..."
			// if ((size_ = recv(sock_, raw_data, buff_size_, MSG_NOSIGNAL)) >= 9) {
			// 	ROS_INFO("CORRECT CALL RESPONSE PACKET RECEIVED, [CURRENT: %d bytes / MINIMUM: %d bytes]", (int)size_, (int)sizeof(obu_tcp_header_t));
			// 	msgs.append((char *)raw_data, size_);
			// 	if (msgs.size() != 0){
			// 		obu_tcp_header_t *header = (obu_tcp_header_t*)&msgs[0];
			// 		unsigned short packet_type = header->packet_type;

			// 	}
			// }
			// while(receve){ //3초 경과하여도 콜 응답 메시지를 못 받으면 동일한 콜에 대하여 재요청함
			// if recv()
			
			// CallRequestMessage(mission_id);



			// }
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND CALL REQUEST PACKET");
			return false;
		}


	}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_;

	CRC16 crc16_;

	sockaddr_in obu_addr_;
	std::string odom_, obu_ip_; //odom topic name is "odom_"
	std::string total_packet;
	int obu_port_, sock_, buff_size_, size_, rate_;
	uint8_t sequence_;

	std::vector<nav_msgs::Odometry> odom_buf_;	

	std::string* pay = NULL;//"3C39020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400200004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007010008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0100A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560401600004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A00004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0100E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D90041660407876";

	// CHECK FLAG PARAMETER
	int is_connect_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;

	return 0;
}
