////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TODOs
//		2. subscriber하나만 만들고 callback을 하다가 odom이 들어오면 subscriber을 하나 더 만들어서 spin()을 하는 방식이 가능한지 확인하기.
//			//확인하는 중! ROSBAG DATA 필요함.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//C++
#include <iostream>
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

//
#include "obu_message.h"
// #include "sample_message.h"

#include "MessageFrame.h"
#include "dsrc_msg_id.h"

#include "libcrc16.h"

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class Call_List_Client {
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_, mission_sub_;

	CRC16 crc16_;

	sockaddr_in obu_addr_;
	std::string obu_ip_, total_packet, operation_env; 
	int obu_port_, sock_, buff_size_, size_, rate_;
	uint8_t sequence_;

	std::vector<nav_msgs::Odometry> odom_buf_;	

	std::string pay = "3C39020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400200004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007010008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0100A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560401600004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A00004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0100E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D90041660407876";

	int is_connect_;

public:	
	Call_List_Client(void):
		private_nh_("~")	
	{
		initParams();
		
		odom_sub_ = nh_.subscribe("gps_odom", 1, &Call_List_Client::odomCallback, this); // sends current position to V2X server
		
		ROS_INFO("CALL LIST CLIENT NODE INITIALIZED");
		while (odom_buf_.size() < 10) {
			static int jaw = 0;
			ROS_INFO("WAITING FOR INITIAL VEHICLE ODOMETRY MESSAGE");
			ros::spinOnce();
			std::cout<<"count:"<<jaw++<<"| odom size is: "<<odom_buf_.size()<<std::endl;
			sleep(1);
		}

		mission_sub_ = nh_.subscribe("vehicle_speed", 1, &Call_List_Client::missionCallback, this); //

	}


	// bool practice(){
	// 	std::string pay_trial = string_to_hex(pay);
	// 	ROS_INFO("NUMBER OF CALLS: %d", (int)pay_trial[0]);
	// 	ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)pay_trial[1]);
	// 	ROS_INFO("NUMBER OF IRREGULARS: %d", (int)pay_trial[2]);

	// 	std::vector<std::string> calls;
	// 	int callData_size = 40;

	// 	for (int i=0; i<(int)pay_trial[0]; i++) {
	// 		// std::cout<<"-\nmission no."<<i+1<<" is"<<std::endl;
	// 		std::string call = pay_trial.substr(3 + i * callData_size, callData_size);
	// 		int id = (int)call[0];
	// 		int m_status = (int)call[1]; //0x00 대기, 0x01 완료
	// 		int m_types = (int)call[2]; // 0001: 좌회전, 0010: 터널, 0100: no HD map, 1000: 비정현
	// 		// 참고하기: https://www.geeksforgeeks.org/check-whether-k-th-bit-set-not/
	// 		std::bitset<8> m_bit = m_types; 
	// 			if (m_bit[0] == true){ROS_INFO("\t- left turn mission");} 
	// 			if (m_bit[1] == true){ROS_INFO("\t- tunnel	 mission");} 
	// 			if (m_bit[2] == true){ROS_INFO("\t- no HD map mission");} 
	// 			if (m_bit[3] == true){ROS_INFO("\t- obstacle	 mission");} 
	// 		unsigned short m_score;
	// 		unsigned short m_distance;
	// 		double slat;
	// 		double slong;
	// 		double elat;
	// 		double elong;
	// 		memcpy(&m_score, &call[3], 2);
	// 		memcpy(&m_distance, &call[5], 2);
	// 		uint8_t m_irregualr_id = (int)call[7];
	// 		memcpy(&slat, &call[8], 8);
	// 		memcpy(&slong, &call[16], 8);
	// 		memcpy(&elat, &call[24], 8);
	// 		memcpy(&elong, &call[32], 8);

	// 		std::cout<<"mission id is : "<<id<<std::endl;
	// 		std::cout<<"mission status (0: 대기, 1:완료) is : "<<m_status<<std::endl;
	// 		ROS_INFO("score is : %d",m_score);
	// 		ROS_INFO("distance is : %d",m_distance);
	// 		ROS_INFO("start lat is %f",slat);
	// 		ROS_INFO("start long is %f",slong);
	// 		ROS_INFO("end lat is %f",elat);
	// 		ROS_INFO("end long is %f",elong);
	// 	}		

	// 	return false;
	// }

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		private_nh_.getParam("test_env", operation_env);
		sock_, size_, is_connect_ = -1;
		sequence_ = 0;
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

	void checkResponse(std::string packet) {
		while (packet.size() != 0) {

				// std::cout<<"\npacket size received is: "<< (int)packet.size() <<std::endl;
				// std::cout<<"original total_packet size is: "<< (int)total_packet.size() <<std::endl;
			total_packet = total_packet + packet;
				// std::cout<<"new total_packet size is: "<< (int)total_packet.size() <<std::endl;
			
			obu_tcp_header_t *header = (obu_tcp_header_t *)&total_packet[0];
			unsigned short packet_type = header->packet_type;


			

			std::string payload, checksum;
			payload.append(total_packet, sizeof(obu_tcp_header_t), header->payload_size);
			checksum.append(total_packet,sizeof(obu_tcp_header_t)+header->payload_size,2);
			// ASSERTION 2: CHECK IF CHECKSUM MATCHES
				unsigned char tmp_payload_arr[header->payload_size];
				memcpy(tmp_payload_arr,&payload,sizeof(header->payload_size));
				uint16_t check_result = crc16_.calcCRC16(tmp_payload_arr, header->payload_size);
				std::string tmp_crc_arr;
				// memcpy(tmp_crc_arr, &check_result,2);
				if( tmp_crc_arr == checksum){
					ROS_INFO("PAYLOAD MESSAGE IS TRUSTED");
				} else {
					ROS_ERROR("PAYLOAD MESSAGE IS INACCURATE");
					total_packet.clear();
					break;
				}
			
			// REAL MESSAGE PARSING
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
					ros::shutdown();
				}
			}
			else if (packet_type == 0x1335) {
				ROS_INFO("[PACKET TYPE: 0x%2x] LOCATION ERROR NOTIFICATION MESSAGE RECIEVED", packet_type);
				unsigned char error_code = payload[0];
				
				if (error_code == 0x01) ROS_ERROR("[ERROR CODE: 0x%x] LATITUDE OF VEHICLE IS OUT OF BOUND", error_code);
				else if (error_code == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] LONGITUDE OF VEHICLE IS OUT OF BOUND", error_code);
				else if (error_code == 0x04) ROS_ERROR("[ERROR CODE: 0x%x] ELEVATION OF VEHICLE IS HIGHER THAN RSU", error_code);
				else if (error_code == 0x08) ROS_ERROR("[ERROR CODE: 0x%x] HEADING VALUE IS OVER 360", error_code);
				else if (error_code == 0x10) ROS_ERROR("[ERROR CODE: 0x%x] LOCATION INFORMATION NOT RECEIVED WITHIN 5 SECONDS AFTER CONNECTION ESTABLISHED", error_code);
				else if (error_code == 0x20) ROS_ERROR("[ERROR CODE: 0x%x] LOCATION INFORMATION NOT RECEIVED FOR 5 SECONDS ", error_code);
				else if (error_code == 0x40) ROS_ERROR("[ERROR CODE: 0x%x] LOCATION PACKET SENT WITH A PERIOD BELOW 50 MILISECONDS", error_code);
				else if (error_code == 0x80) ROS_ERROR("[ERROR CODE: 0x%x] LOCATION PACKET SENT WITH A PERIOD ABOVE 200 MILISECONDS", error_code);
				else ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED ERROR CODE, MAYBE CAUSED BY PARSING ERROR", error_code);
				ros::shutdown();
			}
			else if (packet_type == 0x1136) {
				ROS_INFO("[PACKET TYPE: 0x%2x] CALL LIST MESSAGE RECIEVED", packet_type);


				//// PARSING PAYLOAD /// 필요없으면 Comment해서 없애도됨
				print_call(payload); 

				// STEP 1: // CHOOSE WHICH MISSION TO PROCEED : MISSION PLANNING ALGORITHM
				int mission_id = 60; // FOR NOW... CHOOSE THE CLOSEST STARTING POINT
				// int priority_mission_id = choose_mission(payload);
				
				// STEP 2: // CALL REQUEST MESSAGE (NOTIFY CHOSEN MISSION)
				CallRequestMessage(mission_id);
				
				// while(notified == false){
					
				// 	sleep(3);
				// }

			}
			else if (packet_type == 0x1338) { // 2.3.2.3 콜 응답(콜 매칭) 메시지Call Response-Call Matching Message
				ROS_INFO("[PACKET TYPE: 0x%2x] CALL RESPONSE-CALL MATCHING MESSAGE RECIEVED", packet_type);
				
				int matchingCalld = (int)payload[0];
				ROS_INFO("the requested mission id is %d",matchingCalld);
				unsigned char response = payload[1]; 
				unsigned char error_code = payload[2];
				if (response == 0x00) {
					ROS_INFO("MISSION REQUEST APPROVED");
				} else {
					ROS_ERROR("MISSION REQUEST DENIED");
					if (response == 0x01) ROS_ERROR("[ERROR CODE: 0x%x] penalty-time(미션 포기) 중 요청한 경우",response);
					else if (response == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] COMPETITION IS PAUSED, BUT YOU ASKED FOR A MISSION REQUEST", response);
					else if (response == 0x03) ROS_ERROR("[ERROR CODE: 0x%x] MISSION MATCH SUCCESSFUL, STOP ASKING FOR ANOTHER MISSION BEFORE COMPLETING THIS ONE", response);
					else if (response == 0x04) ROS_ERROR("[ERROR CODE: 0x%x] THIS MISSION IS COMPLETED (S/U/W), YOU CANNOT ASK FOR IT AGAIN", response);
					else if (response == 0x05) ROS_ERROR("[ERROR CODE: 0x%x] MISSION MATCH SUCCESSFUL, STOP AKSING FOR THE SAME MISSION", response);
					else if (response == 0x06) ROS_ERROR("[ERROR CODE: 0x%x] THIS MISSION WAS TAKEN BY ANOTHER TEAM", response);
					else if ((response == 0x10) && (error_code != 0x10)){
						if (error_code == 0x11) ROS_ERROR("[ERROR CODE: 0x%x] V2X 서버가 콜 리스트를 송출하기 전에 요청을 한 경우", error_code);
						else if (error_code == 0x12) ROS_ERROR("[ERROR CODE: 0x%x] requestCallID의 값이 0x00 이거나 0x3C를 초과하는 경우", error_code);
						ros::shutdown();
					}
				}					
			}
			else if (packet_type == 0x133A) { //2.3.2.5 승차 완료 확인 메시지Getting On Confirm Message
				ROS_INFO("[PACKET TYPE: 0x%2x] PASSENGER GETTING ON THE CAR MESSAGE RECIEVED", packet_type);
				int matchingCalld = (int)payload[0];

				unsigned char confirmResult = payload[1];
				
				//MISSION STATE CHANGE
				if (confirmResult == 0x00){ 			// 성공
					ROS_INFO("PICKUP MISSION PASSED");
					//mission_state = dropoff
				} else if (confirmResult == 0x01){		// 실패
					ROS_INFO("PICKUP MISSION FAILED");
					//mission_state = new_mission?
				}
				
				unsigned char error_code = payload[2];
				if ((error_code != 0x00) && (error_code != 0x01)) {
					if (error_code == 0x21) ROS_ERROR("[ERROR CODE: 0x%x] MATCHING CALL ID NOT WITHIN 0x00 AND 0x3C BOUND", error_code);
					else if (error_code == 0x22) ROS_ERROR("[ERROR CODE: 0x%x] LATITUDE  OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x23) ROS_ERROR("[ERROR CODE: 0x%x] LONGITUDE OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x24) ROS_ERROR("[ERROR CODE: 0x%x] NOT THE MISSION ID YOU CHOSE TO DO", error_code);
					else if (error_code == 0x25) ROS_ERROR("[ERROR CODE: 0x%x] MISSION IS NOT YET MATCHED, BUT YOU SENT A CHECK-PICKUP MESSAGE", error_code);
					else if (error_code == 0x26) ROS_ERROR("[ERROR CODE: 0x%x] REFREE CONFIRMED PICKUP, BUT YOU ARE SENDING THE CHECK-PICKUP MESSAGE AGAIN", error_code);
					else if (error_code == 0x27) ROS_ERROR("[ERROR CODE: 0x%x] REFEREE WILL CONFIRM PICKUP, BUT YOU ARE SENDING A CHECK-PICKUP MESSAGE AGAIN", error_code);
					else if (error_code == 0x28) ROS_ERROR("[ERROR CODE: 0x%x] REFEREE CONFIRMED DROPOFF, BUT YOU ARE SENDING A CHECK-PICKUP MESSAGE", error_code);
					else ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED ERROR CODE, MAYBE CAUSED BY PARSING ERROR", error_code);
					ros::shutdown();
				}
			}
			else if (packet_type == 0x133C) { //2.3.2.7 하차 완료 확인 메시지Getting Off Confirm Message
				ROS_INFO("[PACKET TYPE: 0x%2x] PASSENGER GETTING OFF THE CAR MESSAGE RECIEVED", packet_type);
				int matchingCalld = (int)payload[0];

				unsigned char confirmResult = payload[1];
				
				//MISSION STATE CHANGE
				if (confirmResult == 0x00){ 			// 성공
					ROS_INFO("DROPOFF MISSION PASSED");
					//mission_state = move_a_bit
				} else if (confirmResult == 0x01){		// 실패
					ROS_INFO("DROPOFF MISSION FAILED");
					//mission_state = new_mission?
				}
				
				unsigned char error_code = payload[2];
				if ((error_code != 0x00) && (error_code != 0x01)) {
					if (error_code == 0x31) ROS_ERROR("[ERROR CODE: 0x%x] MATCHING CALL ID NOT WITHIN 0x00 AND 0x3C BOUND", error_code);
					else if (error_code == 0x32) ROS_ERROR("[ERROR CODE: 0x%x] LATITUDE  OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x33) ROS_ERROR("[ERROR CODE: 0x%x] LONGITUDE OF VEHICLE IS OUT OF BOUND", error_code);
					else if (error_code == 0x34) ROS_ERROR("[ERROR CODE: 0x%x] NOT THE MISSION ID YOU CHOSE TO DO", error_code);
					else if (error_code == 0x35) ROS_ERROR("[ERROR CODE: 0x%x] MISSION IS NOT YET MATCHED, BUT YOU SENT A CHECK-DROPOFF MESSAGE", error_code);
					else if (error_code == 0x36) ROS_ERROR("[ERROR CODE: 0x%x] REFEREE CONFIRMED DROPOFF, BUT YOU ARE SENDING THE CHECK-DROPOFF MESSAGE AGAIN", error_code);
					else if (error_code == 0x37) ROS_ERROR("[ERROR CODE: 0x%x] REFEREE WILL CONFIRM DROPOFF, BUT YOU ARE SENDING A CHECK-DROPOFF MESSAGE AGAIN", error_code);
					else if (error_code == 0x38) ROS_ERROR("[ERROR CODE: 0x%x] REFEREE DID NOT CONFIRM PICKUP, BUT YOU ARE SENDING A CHECK-DROPOFF MESSAGE", error_code);
					else ROS_ERROR("[ERROR CODE: 0x%x] UNDEFINED ERROR CODE, MAYBE CAUSED BY PARSING ERROR", error_code);
					ros::shutdown();
				}
			}
			else if (packet_type == 0x133D) { //2.3.2.8 미션 완료 결과 메시지Mission-Complete Result Message
				ROS_INFO("[PACKET TYPE: 0x%2x] MISSION COMPLETE MESSAGE RECIEVED!! >", packet_type);
				int matchingCalld = (int)payload[0];

				unsigned char confirmResult = payload[1];
				
				//MISSION STATE CHANGE
				if (confirmResult == 0x00){ 			// 성공
					ROS_INFO("DROPOFF MISSION PASSED");
					//mission_state = CHOOSE_NEW_MISSION
				} else if (confirmResult == 0x01){		// 실패
					ROS_INFO("DROPOFF MISSION FAILED");
					//mission_state = new_mission?
				}
			}
			else {
				ROS_INFO("[PACKET TYPE: 0x%2x] UNDEFINED MESSAGE RECIEVED", packet_type);
			}

			total_packet.clear();
			//packet.erase(0, sizeof(obu_tcp_header_t) + payload.size()); //불필요한것인가? 지울까?
		}
	}

	void print_call(const std::string &payload){
		ROS_INFO("NUMBER OF CALLS: %d", (int)payload[0]);
		ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)payload[1]);
		ROS_INFO("NUMBER OF IRREGULARS: %d", (int)payload[2]);

		int callData_size = 40;
		int irregularData_size = 65;

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
			std::string irregular_call = payload.substr(3+60*callData_size + i * irregularData_size, irregularData_size);
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

	}

	int choose_mission(const std::string &payload){
		int chosen_mission;

		return chosen_mission;
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
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND CALL REQUEST PACKET");
			return false;
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
		
		int sentisize = -1;

		if (sentisize = send(sock_, location_packet, sizeof(location_packet), 0) > 0) {
			double curr_odom_time = ros::Time::now().toSec();
			sequence_++;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND LOCATION PACKET, PLEASE CHECK CONNECTION STATUS");
			return false;
		}
	}
	
	void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
		odom_buf_.emplace_back(*odom_msg);
		odom_buf_.back().header.stamp = ros::Time::now();
		static int hello = 0;
		std::cout<<hello<<" : inside odomCallback!"<<std::endl;
		hello++;
	}

	void missionCallback(const std_msgs::Float64 &speed) { 
		static int my = 0;
		std::cout<<speed<<" <- this is the speed| count: "<<my<<std::endl;
		my++;
	}

};

int main(int argc, char** argv) {

	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;	

	ros::spin();

	return 0;
}
