// TODOs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		5. BIT 정보를 shift함으로써 알아내는 방법.. 출처있음
//
//		7. ros::shutdown()하고 roslaunch가 다시 켜지는지 확인하기. TCP CONNECTION이 끊긴 상황에서는 다시금 CONNECTION이 일어날 수 있도록 해야함.
//			a) 목욜에 확인해보기! 그러면 tcp connection을 잃어도 다시 connection이 일어날것이기 때문에
//
//		8. recv를 3초로 제한하는 방법-> 3초동안 답이 없으면 다시 선택된 미션 send하기
//			a) 이건 도무지 모르겠음 ㅋㅋ..
//
//		9. each payload의 checksum을 확인하기. -> recv msg하면 매번 실행하기. 실패한다면 그 메시지 신뢰하지 않고 그 다음 callback recv으로 가기.
//			//확인하는 중!
//
//		11. Mission State Machine으로 만들기
//			a) OFFLINE되는 버전 만들기
//			b) ONLINE일 때도 되는 버전 만들기
//
//		12. 하고 싶은 미션 순서를 buffer로 만들어서 순차적으로 행하기
//
// 		13. Timer Callback을 만들어서 100 ms 마다 보내지는지 확인하기! Vs. thread쓰는 것 밖에 없다.
//
//		14. Moving Average 로 속도 주기
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DONEs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		1. 현재 위치 보내는 odomCallback()에서 heading과 speed를 구하는 방식에 대한 고민
//			a) ||||나가리|||| 방향성 1: for each info (heading, speed), Callback을 만든다.
//				i) 이게 가장 좋을 것 ... 굳이 계산하는 것이 일인듯....
//			b) ||||나가리|||| 방향성 2: odomCallback()에서 받는 위치를 이용하여 heading과 speed를 구한다.
//			c) 알고 보니 inspva에서 모든 정보를 제공하고 있었다...
//		2. subscriber하나만 만들고 callback을 하다가 odom이 들어오면 subscriber을 하나 더 만들어서 spin()을 하는 방식이 가능한지 확인하기.
//			//확인하는 중! ROSBAG DATA 필요함.
//				가능함! 매우 유용하네!
//		3. OFFLINE일 때 돌아가는 test가능한 function으로 만들기.
//			a) payload로 미션 리스트는 가지고 있음. 이런 상황에서 1) 미션을 고르고 2) 미션을 시작과 끝 좌표를 쏴주기 3) OFFLINE MISSION STATE MACHINE 만들기
//				// 지금 당장 하기!
//		4. For each packet type, 에러가 없다면, payload를 가지고 parsing하고 등등 그 후에 해야할 과정들을 해야하는 function을 하나씩 만들기.
//			a) 현재 8.1a 프로토콜까지는 한듯! -> 확인하는 방법이 있을까?
//		6. lat, long을 UTM으로 변환하고, offset도 subscribe해서 UTM에 offset만큼 뺴기.
//			ans) gpsConv() function으로 구현했음
//		10. gps_odom을 받지 말고(which is in UTM and Offset)... inspva를 그대로 받아서 위치 정보를 서버에 올리기
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
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
#include <numeric>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <thread>
#include <mutex>
#include <fstream>
#include <bitset>
#include <algorithm>
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

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class Call_List_Client {
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber mission_sub_, pos_alert_sub_;
	ros::Subscriber mapgen_sub_;
	// ros::Subscriber globalpath_sub_;
	ros::Publisher  pickup_pub_, dropoff_pub_, target_pub_, currpose_pub_;
	ros::Publisher 	choose_mission_pub_;

	ros::Timer pos_alert_timer_;

	CRC16 crc16_;

	sockaddr_in obu_addr_;
	std::string obu_ip_, total_packet;
	int obu_port_, sock_, buff_size_, size_, rate_;
	double dist_th_, time_th_;
	ros::Time time_;
	uint8_t sequence_;
	uint8_t sequence_location;
	const std::string our_zone = "52N";

	double curr_odom_time = ros::Time::now().toSec();
	double prev_odom_time = ros::Time::now().toSec();

	std::thread th1;

	// send every 100 ms
	unsigned short send_heading = 0;
	double north_vel = 0;
	double east_vel = 0;
	unsigned char send_speed = 0;

	double offset_x = 0;
	double offset_y = 0;

	double curr_pos_lat = 0;
	double curr_pos_long = 0;
	float  curr_pos_z = 0;
	float curr_speed = -1.0;

	struct Mission {
		int id;
		double start_xpos;
		double start_ypos;
		double goal_xpos;
		double goal_ypos;
		double my_priority_score;
		// Mission (int a, double b, double c, double d, double e, double f): id(a), start_xpos(b), start_ypos(c), goal_xpos(d), goal_ypos(e), my_priority_score(f) {}
	};
	struct Mission cur_mission_;
	int selected_mission;
	std::vector<int> temp_mission_lists;

	int key = 0;

	struct less_than_key{
		inline bool operator() (const struct Mission & struct1, const struct Mission & struct2)
		{
			return (struct1.my_priority_score < struct2.my_priority_score);
		}
	};

	//BUFFER
	std::vector<struct Mission> mission_buf_;
	std::vector<double> time_buf_;
	std::vector<float> speed_buf_;

	// V2X MISSION PAYLOAD for <V2X OFFLINE>
	std::string pay = "3C39020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400200004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007010008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0100A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560401600004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A00004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0100E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D90041660407876";

	// FLAGS
	int is_connect_;
	bool is_priority_init_;
	bool is_reach_;
	bool is_start_;
	bool is_end_;
	bool is_time_init_;
	bool is_initialized_;
	bool is_rehersal_;
	bool is_v2x_on_;
	bool use_custom_mission_;
	bool mapinfo_avail;
	bool is_during_mission_ = false;

	enum Mission_State {listening, choosing_mission, start_mission, going_to_start_pos, pickingup, start_dropoff, going_to_end_pos, droppingoff, no_more_missions} mission_state;


public:
	Call_List_Client(void):
		private_nh_("~")
	{
		initParams();

		// CALLBACK 2: sends current position to V2X server
		pos_alert_sub_ = nh_.subscribe("inspva",1, &Call_List_Client::posealertCallback, this);
		while ( curr_pos_lat == 0 && curr_pos_long == 0 && curr_pos_z == 0) {
			ROS_ERROR("TURN ON NOVATEL! INSPVA NEEDED TO ESABLISH V2X CONNECTION REQUEST!");
			ros::spinOnce();
			sleep(1);
		}

		memset(&obu_addr_, 0, sizeof(obu_addr_));
		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_port = htons(uint32_t(obu_port_));

		if (inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr) <= 0) {
			ROS_ERROR("FAILED TO CONVERT IP ADDRESS TO BINARY TYPE");
		}

		if ((sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
			ROS_ERROR("FAILED TO CREATE SOCKET FILE DESCRIPTOR");
		}

	}

	//// 보험용!
	std::string static convertArrToHexStr(char *data, int len) {
		std::string s(len * 2, ' ');
		for (int i=0;i<len;++i) {
			s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
			s[2 * i + 1] = hexmap[data[i] & 0x0F];
		}
		return s;
	}


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


	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("sleep_rate", rate_);
		private_nh_.getParam("is_v2x_on_", is_v2x_on_);
		private_nh_.getParam("is_rehersal_", is_rehersal_);
		private_nh_.getParam("use_custom_mission_", use_custom_mission_);

		private_nh_.getParam("distance_threshold", dist_th_);
		private_nh_.getParam("time_threshold", time_th_);

		is_reach_, is_start_, is_priority_init_, is_time_init_, is_initialized_ = false;
		is_end_ = true;
		sock_, size_, is_connect_ = -1;
		mission_state = listening;
		sequence_ = 0;
		sequence_location=0;
	}


	void sendCurrentLocation(double latitude, double longitude, float elevation, unsigned short heading, unsigned char speed) {

		double prev_odom_time = ros::Time::now().toSec();
		double curr_odom_time = ros::Time::now().toSec();

		unsigned char location_packet[34];
		memset(location_packet, 0x00, sizeof(location_packet));
		location_packet[0] = 0x35;
		location_packet[1] = 0x31;
		location_packet[2] = sequence_location;
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

		std::cout<<"2||"<<"lat is: "<<latitude<<" long is: "<<longitude<<" elev is: "<<elevation<<" heading is: "<<heading<<" speed is: "<<(int)speed<<std::endl;
		std::string cc = convertArrToHexStr((char*)location_packet,34);
		std::cout<<"cc: "<<cc<<std::endl;
		if (sentisize = send(sock_, location_packet, sizeof(location_packet), 0) > -1) {
			//time count
			curr_odom_time = ros::Time::now().toSec();
			double t = curr_odom_time - prev_odom_time;
			// ROS_INFO("\n[loop time (100ms) : %.4f ]SEND ACCESS LOCATION PACKET SUCCESSFULLY", t*1000);
			prev_odom_time = curr_odom_time;
			sequence_location++;
		}else {
			ROS_ERROR("FAILED TO SEND LOCATION PACKET, PLEASE CHECK CONNECTION STATUS");
		}

		usleep(100000); //100 miliseconds 주기로 메시지가 보내져야함 (50ms ~ 200ms 사이)
	}

	void posealertCallback(const novatel_gps_msgs::InspvaPtr& gpsHandler){
		// update current position
		std::mutex mtx;
		mtx.lock();
			curr_pos_lat = gpsHandler->latitude;
			curr_pos_long = gpsHandler->longitude;
			curr_pos_z = gpsHandler->height;
			send_heading = (unsigned short) gpsHandler->azimuth;
			north_vel = gpsHandler->north_velocity;
			east_vel = gpsHandler->east_velocity;
			send_speed = (unsigned char) sqrt( east_vel*east_vel + north_vel*north_vel )*3.6; // m/s to km/h conversion
		mtx.unlock();

		sendCurrentLocation(curr_pos_lat,curr_pos_long,curr_pos_z,send_heading,send_speed);
	}


};

int main(int argc, char** argv) {

	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;

	ros::spin();

	return 0;
}
