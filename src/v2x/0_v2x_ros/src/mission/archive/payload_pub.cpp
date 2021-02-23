// TODOs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	publish payload
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "path_msgs/Map.h"
#include "path_msgs/Trajectory.h"
#include "novatel_gps_msgs/Inspva.h"
#include "mission_msgs/mission.h"
#include "mission_msgs/mission_list.h"

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
// LLtoUTM or UTMtoLL
#include "conversions.h"
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

class Call_List_Client {
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Publisher 	choose_mission_pub_;
	ros::Subscriber pos_alert_sub_;


	// V2X MISSION PAYLOAD for <V2X OFFLINE>
	std::string pay = "3C39020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400200004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007010008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0100A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560401600004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A00004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0100E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D90041660407876";
	

public:	
	Call_List_Client(void):
		private_nh_("~")	
	{
		
		choose_mission_pub_ = nh_.advertise<mission_msgs::mission_list>("mission_list", 1);
		pos_alert_sub_ = nh_.subscribe("inspva",1, &Call_List_Client::posealertCallback, this);
		
		ROS_INFO("CALL LIST CLIENT NODE INITIALIZED");
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



	void print_call(const std::string &payload){
		ROS_INFO("NUMBER OF CALLS: %d", (int)payload[0]);
		ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)payload[1]);
		ROS_INFO("NUMBER OF IRREGULARS: %d", (int)payload[2]);

		int callData_size = 40;
		int irregularData_size = 65;

		for (int i=0; i<(int)payload[0]; i++) {
			std::string call = payload.substr(3 + i * callData_size, callData_size);
			int id = (int)call[0];
			bool m_status = (bool)call[1]; //0x00 대기, 0x01 완료
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



	void posealertCallback(const novatel_gps_msgs::InspvaPtr& gpsHandler){
		
		double send_lat = gpsHandler->latitude;
		double send_long = gpsHandler->longitude;
		float send_elev = gpsHandler->height;
				
		static std::string newpay = hex_to_string(pay);
		// print_call(newpay);
		send(newpay, send_lat, send_long);
		
		sleep(1); 
	}

	void send(const std::string & payload, const double lat, const double longi){
		ROS_INFO("NUMBER OF CALLS: %d", (int)payload[0]);
		ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)payload[1]);
		ROS_INFO("NUMBER OF IRREGULARS: %d", (int)payload[2]);

		mission_msgs::mission_list buffer;


		int callData_size = 40;
		int irregularData_size = 65;
		
		for (int i=0; i<(int)payload[0]; i++) {

			mission_msgs::mission mission_impossible;
			mission_impossible.curr_pos.x = lat;
			mission_impossible.curr_pos.y = longi;

			std::string call = payload.substr(3 + i * callData_size, callData_size);
			int id = (int)call[0];
			bool m_status = (bool)call[1]; //0x00 대기, 0x01 완료
			int m_types = (int)call[2]; // 0001: 좌회전, 0010: 터널, 0100: no HD map, 1000: 비정현
			std::bitset<8> m_bit = m_types; 
				if (m_bit[0] == true){
					// ROS_INFO("\t- left turn mission");
					mission_impossible.left_turn_mission = true;
				} 
				if (m_bit[1] == true){
					// ROS_INFO("\t- tunnel	 mission");
					mission_impossible.tunnel_mission = true;
				} 
				if (m_bit[2] == true){
					// ROS_INFO("\t- no HD map mission");
					mission_impossible.no_HD_map_mission = true;
				} 
				if (m_bit[3] == true){
					// ROS_INFO("\t- obstacle	 mission");
					mission_impossible.obstacle_mission = true;
				} 
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

			// std::cout<<"mission id is : "<<id<<std::endl;
			mission_impossible.mission_id = id;
			// std::cout<<"mission status (0: 대기, 1:완료) is : "<<m_status<<std::endl;
			mission_impossible.mission_state = m_status;
			// ROS_INFO("score is : %d",m_score);
			mission_impossible.score = m_score;
			// ROS_INFO("distance is : %d",m_distance);
			mission_impossible.dist = m_distance;
			// ROS_INFO("start lat is %f",slat);
			// ROS_INFO("start long is %f",slong);
			mission_impossible.start_pos.x = slat;
			mission_impossible.start_pos.y = slong;
			// ROS_INFO("end lat is %f",elat);
			// ROS_INFO("end long is %f",elong);
			mission_impossible.end_pos.x = elat;
			mission_impossible.end_pos.y = elong;

			buffer.mission_lists.push_back(mission_impossible);

		}				


		for (int i=0; i < 2; i++){

			mission_msgs::mission mission_impossible;
			mission_impossible.curr_pos.x = lat;
			mission_impossible.curr_pos.y = longi;

			std::string irregular_call = payload.substr(3+60*callData_size + i * irregularData_size, irregularData_size);
			int id = (int)irregular_call[0];
			mission_impossible.mission_id = id + 60;
			// std::cout<<"\nirregular object id is : "<<id<<std::endl;

			double lat1, long1, lat2, long2, lat3, long3, lat4, long4;
			memcpy(&lat1,&irregular_call[1],8);
			memcpy(&long1,&irregular_call[9],8);
			memcpy(&lat2,&irregular_call[17],8);
			memcpy(&long2,&irregular_call[25],8);
			memcpy(&lat3,&irregular_call[33],8);
			memcpy(&long3,&irregular_call[41],8);
			memcpy(&lat4,&irregular_call[49],8);
			memcpy(&long4,&irregular_call[57],8);

			// std::cout<<"irregular object's coordinate is ..."<<std::endl;
			// ROS_INFO("latitude  1 is : %f",lat1);
			// ROS_INFO("longitude 1 is : %f",long1);
			// ROS_INFO("latitude  2 is : %f",lat2);
			// ROS_INFO("longitude 2 is : %f",long2);
			mission_impossible.start_pos.x = (lat1 + lat2)/2;
			mission_impossible.start_pos.y = (long1 + long2)/2;
			// ROS_INFO("latitude  3 is : %f",lat3);
			// ROS_INFO("longitude 3 is : %f",long3);
			// ROS_INFO("latitude  4 is : %f",lat4);
			// ROS_INFO("longitude 4 is : %f",long4);
			mission_impossible.end_pos.x = (lat3 + lat4)/2;
			mission_impossible.end_pos.y = (long3 + long4)/2;
			buffer.mission_lists.push_back(mission_impossible);
		}

		choose_mission_pub_.publish(buffer);
	}
	
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;	
	
	ros::spin();

	return 0;
}
