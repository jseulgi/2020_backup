// TODOs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		7. ros::shutdown()하고 roslaunch가 다시 켜지는지 확인하기. TCP CONNECTION이 끊긴 상황에서는 다시금 CONNECTION이 일어날 수 있도록 해야함.
//			a) 목욜에 확인해보기! 그러면 tcp connection을 잃어도 다시 connection이 일어날것이기 때문에
//
//		8. recv를 3초로 제한하는 방법-> 3초동안 답이 없으면 다시 선택된 미션 send하기
//			a) 이건 도무지 모르겠음 ㅋㅋ.. -> timer을 만들어서 확인하는 것 만들기 -> 하지만 필요할까? 이런경우가 한번도 없었는데?
//
//		12. 하고 싶은 미션 순서를 buffer로 만들어서 순차적으로 행하기 -> subscriber & publisher 만들어야함.
//			-> next version에서 구현하기
//			
//		15. Publisher들을 latch로 만들어야하나?
// 		16. 
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
//		5. BIT 정보를 shift함으로써 알아내는 방법.. 출처있음  -> 굳이할 필요가 없는듯
//		6. lat, long을 UTM으로 변환하고, offset도 subscribe해서 UTM에 offset만큼 뺴기.
//			ans) gpsConv() function으로 구현했음
//		9. each payload의 checksum을 확인하기. -> recv msg하면 매번 실행하기. 실패한다면 그 메시지 신뢰하지 않고 그 다음 callback recv으로 가기.
//			// DON'T NEED
//		10. gps_odom을 받지 말고(which is in UTM and Offset)... inspva를 그대로 받아서 위치 정보를 서버에 올리기
//		11. Mission State Machine으로 만들기
//			a) OFFLINE되는 버전 만들기			[ERASED]
//			b) ONLINE일 때도 되는 버전 만들기	  [DONE]	
// 		13. Timer Callback을 만들어서 100 ms 마다 보내지는지 확인하기! Vs. thread쓰는 것 밖에 없다.
//		14. Moving Average 로 속도 주기
//
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
#include "obj_msgs/Obj.h"
#include "obj_msgs/ObjList.h"
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
// PARSING
#include "obu_message.h"
#include "MessageFrame.h"
#include "dsrc_msg_id.h"
#include "libcrc16.h"

#define SOCKET		true 		//true: socket programming is enabled
#define LOAD_DATA	false		//true: load mission data is enabled

constexpr const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a' , 'b', 'c', 'd', 'e', 'f'};

class Call_List_Client {
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber mission_sub_, pos_alert_sub_, mapgen_sub_;

	ros::Publisher  pickup_pub_, dropoff_pub_, target_pub_, currpose_pub_;
	ros::Publisher 	choose_mission_pub_;  // mission list publishing to 창기형
	ros::Publisher 	two_static_obstacles_pub_;  // 장애물 정보 publish함

	ros::Timer pos_alert_timer_;

	CRC16 crc16_;

	sockaddr_in obu_addr_;
	std::string obu_ip_, total_packet;
	int obu_port_, sock_, buff_size_, size_;
	double dist_th_, time_th_;
	ros::Time time_;
	uint8_t sequence_;
	uint8_t sequence_location;
	const std::string our_zone = "52N";

	double curr_odom_time = ros::Time::now().toSec();
	double prev_odom_time = ros::Time::now().toSec();
	double count5start;
	// double count5curr;

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
	};
	struct Mission cur_mission_;

	int selected_mission;
	std::vector<int> launchfile_mission_list;
	std::vector<int> done_mission_list;
	int mission_alternative;

	//BUFFER
	std::vector<struct Mission> mission_buf_;
	std::vector<int> mission_in_order;
	std::vector<float> speed_buf_{0,};

	#if LOAD_DATA
	// V2X MISSION PAYLOAD for <V2X OFFLINE>
	std::string loaded_pay0 = "3C39020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400200004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007010008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0100A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560401600004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A00004D0565010035B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0000CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320000E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340000290B65010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500006E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600007B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370000A80EB803020F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000008075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0000B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0100E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D90041660407876";
	std::string loaded_pay1007 = "3C3A020100000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040020008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400300026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400400017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040050003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040060000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604007000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040080000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040090002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560400A0002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660400B00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560400C0008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560400D00026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560400E00017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660400F0003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040100100EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604011000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040120000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040130002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040140002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401500000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040160008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560401700026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560401800017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040190003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660401A0100EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560401B000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560401C0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560401D0002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560401E0002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660401F00000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040200008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402100026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402200017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040230003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040240000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604025000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040260000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040270002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040280002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660402900000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC1560402A0008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560402B00026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560402C00017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A2141660402D0003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E1660402E0000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E21560402F000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE3156040300000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA156040310002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D0156040320002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A0061660403300000106190200D4BB783F6EEB4140AF21382EE3156040EE5D83BEF4EA414015ADDC0BCC156040340008410765010135B39602D2EA414091EF52EA12166040357F4C6BD3EA414042D2A755F41560403500026E07BE02000342EBE1CBEA4140D998D71107166040EE5D83BEF4EA4140EA3E00A9CD1560403600017B072F030036E7E099D0EA414020425C39FB15604057ED9A90D6EA4140CCD3B9A214166040370003CC08B803000F81238106EB4140D82C978DCE15604001892650C4EA41403410CB660E166040380000EB04030100E57E87A240EB4140CA897615D2156040ACC43C2B69EB4140A48D23D6E215604039000208075802009EEFA7C64BEB414090BB085314166040D4BB783F6EEB4140AF21382EE31560403A0000F004080100CB2F833122EB41408A5B0531D01560403830B95164EB4140C007AF5DDA1560403B0002B20602020016C26A2C61EB414051A1BAB9F8156040A818E76F42EB4140B952CF82D01560403C0002E1063102004A7EC4AF58EB414009DD2571D6156040BF4692205CEB41405C9198A00616604001134548DDCEEA4140E3344415FE156040BA2D910BCEEA41401FDAC70AFE1560402B6D718DCFEA4140B0E8D66BFA1560408484285FD0EA41408C852172FA156040021749BBD1C7EA4140738577B9081660407C4276DEC6EA4140676490BB081660409FE40E9BC8EA4140F2B567960416604028806264C9EA414015191D9004166040F269";
	std::string loaded_pay = loaded_pay1007;
	#endif
	
	// FLAGS
	int is_connect_;
	bool mapinfo_avail;
	bool is_during_mission_;
	bool count_started;

	enum Mission_State {listening, choosing_mission, start_mission, going_to_start_pos, pickingup, start_dropoff, going_to_end_pos, droppingoff, no_more_missions} mission_state;


public:
	Call_List_Client(void):
		private_nh_("~")
	{
		initParams();
		initSetup();
		ROS_INFO("CALL LIST CLIENT NODE INITIALIZED");
	}

	void initParams(void) {
		private_nh_.getParam("obu_ip", obu_ip_);
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("buffer_size", buff_size_);
		private_nh_.getParam("/mission_list",launchfile_mission_list);

		mission_state = listening;
		sock_, size_, is_connect_ = -1;
		mission_alternative = 0;
		count_started = false;
		is_during_mission_ = false;

		ROS_INFO("THIS IS YAML THE MISSION LIST");
		for (int i=0; i < launchfile_mission_list.size(); i++){
			std::cout<<"mission "<<i+1<<" : "<<launchfile_mission_list[i]<<std::endl;
		}

		ROS_ERROR("INIT COMPLETE!");
	}

	void initSetup(void){
		// PUBLISHERS
		pickup_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pickup_position", 1);
		dropoff_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_position", 1);
		target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
		choose_mission_pub_ = nh_.advertise<mission_msgs::mission>("mission_list", 1);
		currpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
		two_static_obstacles_pub_ = nh_.advertise<obj_msgs::ObjList>("two_static_obstacles", 1, true);

		// CALLBACK 1: OFFSET 가져오기
		mapinfo_avail = false;
		mapgen_sub_ = nh_.subscribe("map_info",1, &Call_List_Client::mapCallback,this);
		while ( !mapinfo_avail ){
			ROS_ERROR("TURN ON MAP GENERATION!");
			ros::spinOnce();
			sleep(1);
		}

		// CALLBACK 2: sends current position to V2X server
		pos_alert_sub_ = nh_.subscribe("inspva",1, &Call_List_Client::posealertCallback, this);
		while ( curr_pos_lat == 0 && curr_pos_long == 0 && curr_pos_z == 0) {
			ROS_ERROR("TURN ON NOVATEL! INSPVA NEEDED TO ESABLISH V2X CONNECTION REQUEST!");
			ros::spinOnce();
			sleep(1);
		}

		#if SOCKET 
		if (!requestConnection()) ros::shutdown();
		#endif

		// 현재 속도 가져오기
			// call state를 바꿔가면서 상황 판단하기
		mission_sub_ = nh_.subscribe("vehicle_speed", 1, &Call_List_Client::missionCallback, this);
		while (curr_speed == -1){
			ROS_ERROR("TURN ON CanNodegen!");
			sleep(1);
			ros::spinOnce();
		}

		// globalpath_sub_ = nh_.subscribe("global_trajectory",1, &Call_List_Client::globalpathCallback, this);

		ROS_ERROR("SETUP COMPLETE!");
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

		unsigned char lat_arr[sizeof(curr_pos_lat)];
		memcpy(lat_arr, &curr_pos_lat, sizeof(curr_pos_lat));
		unsigned char long_arr[sizeof(curr_pos_long)];
		memcpy(long_arr, &curr_pos_long, sizeof(curr_pos_long));
		unsigned char elev_arr[sizeof(curr_pos_z)];
		memcpy(elev_arr, &curr_pos_z, sizeof(curr_pos_z));
		unsigned char payload_arr[20];
		memcpy(payload_arr, lat_arr, sizeof(curr_pos_lat));
		memcpy(&payload_arr[8], long_arr, sizeof(curr_pos_long));
		memcpy(&payload_arr[16], elev_arr, sizeof(curr_pos_z));
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 20);
		unsigned char crc_arr[2];
		memcpy(&permission_packet[9], lat_arr, sizeof(curr_pos_lat));
		memcpy(&permission_packet[17], long_arr, sizeof(curr_pos_long));
		memcpy(&permission_packet[25], elev_arr, sizeof(curr_pos_z));
		memcpy(crc_arr, &check_result, 2);
		permission_packet[29] = crc_arr[0];
		permission_packet[30] = crc_arr[1];

		int sentisize = -1;

		if (sentisize = send(sock_, permission_packet, sizeof(permission_packet), 0) > -1) {
			ROS_INFO("SEND ACCESS PERMISSION PACKET SUCCESSFULLY");
			sequence_++;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND PERMISSION PACKET, PLEASE CHECK CONNECTION STATUS");
			return false;
		}

	}

	void receive_message(){
		char raw_data[buff_size_];
		std::string msgs;
		// recv의 maximum time slot을 3초로 해야함 -> 3초동안 안되면 새로 실행하게끔
		double tt = ros::Time::now().toSec();
		ROS_INFO("now recv()ing!...");
		if ((size_ = recv(sock_, raw_data, buff_size_, MSG_DONTWAIT)) >= 9) {
			tt = ros::Time::now().toSec() - tt;
			ROS_INFO("TIME taken to recv() is : %.4f",tt);
			ROS_INFO("RECEIVED, [CURRENT: %d bytes / MINIMUM: %d bytes]", (int)size_, (int)sizeof(obu_tcp_header_t));
			msgs.append((char *)raw_data, size_);
			checkResponse(msgs);
		}
	}

	void checkResponse(std::string packet) {
		while (packet.size() != 0) {

			total_packet = total_packet + packet;

			obu_tcp_header_t *header = (obu_tcp_header_t *)&total_packet[0];
			unsigned short packet_type = header->packet_type;

			std::string payload, checksum;
			payload.append(total_packet, sizeof(obu_tcp_header_t), header->payload_size);

			// ASSERTION 0: CHECK PACKET TYPE
				if ((packet_type != 0x133F) && (packet_type != 0x1334) && (packet_type != 0x1335) && (packet_type != 0x1136) && (packet_type != 0x1338) && (packet_type != 0x133A) && (packet_type != 0x133C) && (packet_type != 0x133D)){
					ROS_ERROR("[PACKET TYPE: 0x%2x] UNDEFINED MESSAGE RECIEVED", packet_type);
					total_packet.clear();
					packet.clear();
					break;
				}

			// ASSERTION 1: CHECK IF ENOUGH PACKETS ARRIVED
				if ( total_packet.size() < header->payload_size + 9){
					ROS_INFO("[ BAD ]   GOT %d/%d BYTES !",(int)total_packet.size(),(int)header->payload_size + 9);
					packet.clear();
					break;
				} else if ( total_packet.size()  == header->payload_size + 9){
					ROS_INFO("[ GOOD ]  GOT %d/%d BYTES !",(int)total_packet.size(),(int)header->payload_size + 9);
				} if ( total_packet.size() >  header->payload_size + 9){
					ROS_INFO("[ WEIRD ] GOT %d/%d BYTES !",(int)total_packet.size(),(int)header->payload_size + 9);
					// 이런경우가 생기긴하는데 맨 처음에 total_packet에 단순히 packet을 더해서 buffersize 크기 만한 것이 total_packet에 들어가게됨 // 나중에 고치기
				}

			ROS_INFO("Packet Type is: |0x%2x| ",packet_type);

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
				// ASSERTION 3: WAITING FOR SPECIFIC PACKET TYPE: 2.3.2.5 승차 완료 확인 메시지Getting On Confirm Message
				if (is_during_mission_ == true ){
					ROS_WARN("DURING A MISSION! DONT NEED 0x1336 PACKETS, IGNORING MISSION LIST");
					packet.clear();
					total_packet.clear();
					break;
				}

				// save_payload(payload ,"\nMission List!\n");


				// PRINT MISSIONS ONCE & PUTTING MISSIONS IN A BUFFER
				print_call(payload);
				
				// 우선순위한 미션 순서 중에서 가장 첫번째부터 고르게끔 mission_alternative를 0으로 reset하기
				mission_alternative = 0;

				// DONE LISTENING TO MISSION LIST
				mission_state = choosing_mission;

			}
			else if (packet_type == 0x1338) { // 2.3.2.3 콜 응답(콜 매칭) 메시지Call Response-Call Matching Message
				ROS_INFO("[PACKET TYPE: 0x%2x] CALL RESPONSE-CALL MATCHING MESSAGE RECIEVED", packet_type);

				int matchingCalld = (int)payload[0];
				ROS_INFO("the requested mission id is %d",matchingCalld);
				unsigned char response = payload[1];
				unsigned char error_code = payload[2];
				if (response == 0x00) {
					ROS_INFO("MISSION REQUEST APPROVED\n\nyay\n");
					mission_state = start_mission;
				} else {
					ROS_ERROR("MISSION REQUEST DENIED");
					mission_state = listening; //왠만하면 listening state로가지만 아래의 경우에 따라 mission_state가 달라질 수 있음.
					if (response == 0x01) {
						ROS_ERROR("[ERROR CODE: 0x%x] YOU REQUESTED A MISSION DURING THE PENALTY TIME (GAVE UP MISSION)",response);
						// ros::shutdown()//하는게 낫겠지???
					}
					else if (response == 0x02) ROS_ERROR("[ERROR CODE: 0x%x] COMPETITION IS PAUSED, BUT YOU ASKED FOR A MISSION REQUEST", response);
					else if (response == 0x03) {
						ROS_ERROR("[ERROR CODE: 0x%x] MISSION MATCH SUCCESSFUL, STOP ASKING FOR ANOTHER MISSION BEFORE COMPLETING THE MATCHED MISSION", response);
						ROS_ERROR("if you don't know which mission your matched to just ask for the drop of the mission");
						ros::shutdown();
					}
					else if (response == 0x04) {
						ROS_ERROR("[ERROR CODE: 0x%x] THIS MISSION IS COMPLETED (S/U/W), YOU CANNOT ASK FOR IT AGAIN", response);
						done_mission_list.push_back(selected_mission);
						//-----
						mission_alternative++; // choose the next mission in line
						mission_state = choosing_mission;
						//-----
					}
					else if (response == 0x05) {
						ROS_ERROR("[ERROR CODE: 0x%x] MISSION MATCH SUCCESSFUL, STOP AKSING FOR THE SAME MISSION\n Going to \"start_mission\" state", response);
						// 이 미션을 수행하면 되니까 다음 state로 넘어가자
						mission_state = start_mission;
					}
					else if (response == 0x06) {
						ROS_ERROR("[ERROR CODE: 0x%x] THIS MISSION WAS TAKEN BY ANOTHER TEAM", response);
						//-----
						mission_alternative++; // choose the next mission in line
						mission_state = choosing_mission;
						//-----
					}
					else if ((response == 0x10) && (error_code != 0x10)){
						if (error_code == 0x11) ROS_ERROR("[ERROR CODE: 0x%x] V2X 서버가 콜 리스트를 송출하기 전에 요청을 한 경우", error_code);
						else if (error_code == 0x12) ROS_ERROR("[ERROR CODE: 0x%x] requestCallID의 값이 0x00 이거나 0x3C를 초과하는 경우", error_code);
						ros::shutdown();
					}
				}
			}
			else if (packet_type == 0x133A) { //2.3.2.5 승차 완료 확인 메시지Getting On Confirm Message
				ROS_INFO("[PACKET TYPE: 0x%2x] PASSENGER GETTING ON THE CAR MESSAGE RECIEVED", packet_type);
				// save_payload(payload ,"\nPickUp Confirmed!\n");
				int matchingCalld = (int)payload[0];
				unsigned char confirmResult = payload[1];

				//MISSION STATE CHANGE
				if (confirmResult == 0x00){ 			// 성공
					ROS_INFO("PICKUP MISSION PASSED");
					mission_state = start_dropoff;
				} else if (confirmResult == 0x01){		// 실패
					ROS_INFO("PICKUP MISSION FAILED");
					mission_state = listening;
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
				// save_payload(payload ,"\nDropOff Confirmed!\n");
				int matchingCalld = (int)payload[0];

				unsigned char confirmResult = payload[1];

				//MISSION STATE CHANGE
				if (confirmResult == 0x00){ 			// 성공
					ROS_INFO("DROPOFF MISSION PASSED");
					mission_state = listening;
					// key++;
					// if (key == launchfile_mission_list.size()){ mission_state = no_more_missions;}
					is_during_mission_=false;
				} else if (confirmResult == 0x01){		// 실패
					ROS_INFO("DROPOFF MISSION FAILED");
					sleep(5);
					mission_state = listening;
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


			total_packet.clear();
			packet.clear();
			ROS_INFO_STREAM("total packet Cleared");
			//packet.erase(0, sizeof(obu_tcp_header_t) + payload.size()); //불필요한것인가? 지울까?
		}
	}

	int print_call(const std::string &payload){
		if (mission_buf_.size() != 0){
			return 0;
		}
		ROS_INFO("NUMBER OF CALLS: %d", (int)payload[0]);
		ROS_INFO("NUMBER OF AVAILABLE CALLS: %d", (int)payload[1]);
		ROS_INFO("NUMBER OF IRREGULARS: %d", (int)payload[2]);

		int callData_size = 40;
		int irregularData_size = 65;

		struct Mission tmp_mission;

		for (int i=0; i<(int)payload[0]; i++) {
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
			tmp_mission.id = id;
			tmp_mission.start_xpos = slat;
			tmp_mission.start_ypos = slong;
			tmp_mission.goal_xpos = elat;
			tmp_mission.goal_ypos = elong;
			mission_buf_.push_back(tmp_mission);
		}
		
		obj_msgs::ObjList static_object_list;

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

			geometry_msgs::Point pnt1;
			pnt1.x = lat1;
			pnt1.y = long1;
			geometry_msgs::Point pnt2;
			pnt2.x = lat2;
			pnt2.y = long2;
			geometry_msgs::Point pnt3;
			pnt3.x = lat3;
			pnt3.y = long3;
			geometry_msgs::Point pnt4;
			pnt4.x = lat4;
			pnt4.y = long4;
			
			obj_msgs::Obj tmp;
			tmp.bl = pnt1; 
			tmp.br = pnt2;
			tmp.ul = pnt3;
			tmp.ur = pnt4;

			static_object_list.objlist.push_back(tmp);
		}
		two_static_obstacles_pub_.publish(static_object_list);
	}

	int CallRequestMessage(int mission_id){ //선점하고자하는 메시지를 보내는 것, 프로토콜 2.3.2.2임
		unsigned char call_packet[12];

		memset(call_packet, 0x00, sizeof(call_packet));
		call_packet[0] = 0x37;
		call_packet[1] = 0x31;
		call_packet[2] = sequence_;
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
			mission_state = listening;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND CALL REQUEST PACKET");
			return false;
		}
	}

	int StartingPointArriveMessage(int mission_id, double latitude, double longitude, unsigned char speed){ //출발지 도착 완료 메시지Starting-Point Arrive Message, 프로토콜 2.3.2.4임
		unsigned char pickup_packet[29];

		memset(pickup_packet, 0x00, sizeof(pickup_packet));
		pickup_packet[0] = 0x39;
		pickup_packet[1] = 0x31;
		pickup_packet[2] = sequence_;
		pickup_packet[3] = 0x14;
		pickup_packet[4] = 0x00;
		pickup_packet[5] = 0xCE;
		pickup_packet[6] = 0x3B;
		pickup_packet[7] = 0x31;
		pickup_packet[8] = 0x9E;

		unsigned char lat_arr[sizeof(latitude)];
		memcpy(lat_arr, &latitude, sizeof(latitude));
		unsigned char long_arr[sizeof(longitude)];
		memcpy(long_arr, &longitude, sizeof(longitude));
		unsigned char speed_arr[sizeof(speed)];
		memcpy(speed_arr, &speed, sizeof(speed));
		unsigned char payload_arr[18];
		payload_arr[0] = (unsigned char) mission_id;
		memcpy(&payload_arr[1], lat_arr, sizeof(latitude));
		memcpy(&payload_arr[9], long_arr, sizeof(longitude));
		memcpy(&payload_arr[17], speed_arr, sizeof(speed));
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 18);
		unsigned char crc_arr[2];
		memcpy(crc_arr, &check_result, 2);

		pickup_packet[9] = (unsigned char) mission_id;
		memcpy(&pickup_packet[10], lat_arr, sizeof(latitude));
		memcpy(&pickup_packet[18], long_arr, sizeof(longitude));
		memcpy(&pickup_packet[26], speed_arr, sizeof(speed));
		pickup_packet[27] = crc_arr[0];
		pickup_packet[28] = crc_arr[1];

		int sentisize = -1;
		sentisize = send(sock_, pickup_packet, sizeof(pickup_packet), 0);
		ROS_INFO("Sentisize is %d",sentisize);
		ROS_INFO("pickup packet size (should be 29) is %d", (int)sizeof(pickup_packet));

		if (sentisize > -1) {
			ROS_INFO("SEND ARRIVE AT PICKUP LOCATION PACKET SUCCESSFULLY");
			sequence_++;
			mission_state = listening;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND ARRIVE AT PICKUP LOCATION PACKET");
			return false;
		}
	}

	int EndPointArriveMessage(int mission_id, double latitude, double longitude, unsigned char speed){ //목적지 도착 완료 메시지End-Point Arrive Message, 프로토콜 2.3.2.6임
		unsigned char dropoff_packet[29];

		memset(dropoff_packet, 0x00, sizeof(dropoff_packet));
		dropoff_packet[0] = 0x3B;
		dropoff_packet[1] = 0x31;
		dropoff_packet[2] = sequence_;
		dropoff_packet[3] = 0x14;
		dropoff_packet[4] = 0x00;
		dropoff_packet[5] = 0xCE;
		dropoff_packet[6] = 0x3B;
		dropoff_packet[7] = 0x31;
		dropoff_packet[8] = 0x9E;
		unsigned char lat_arr[sizeof(latitude)];
		memcpy(lat_arr, &latitude, sizeof(latitude));
		unsigned char long_arr[sizeof(longitude)];
		memcpy(long_arr, &longitude, sizeof(longitude));
		unsigned char speed_arr[sizeof(speed)];
		memcpy(speed_arr, &speed, sizeof(speed));
		unsigned char payload_arr[18];
		payload_arr[0] = (unsigned char) mission_id;
		memcpy(&payload_arr[1], lat_arr, sizeof(latitude));
		memcpy(&payload_arr[9], long_arr, sizeof(longitude));
		memcpy(&payload_arr[17], speed_arr, sizeof(speed));
		uint16_t check_result = crc16_.calcCRC16(payload_arr, 18);
		unsigned char crc_arr[2];
		memcpy(crc_arr, &check_result, 2);

		dropoff_packet[9] = (unsigned char) mission_id;
		memcpy(&dropoff_packet[10], lat_arr, sizeof(latitude));
		memcpy(&dropoff_packet[18], long_arr, sizeof(longitude));
		memcpy(&dropoff_packet[26], speed_arr, sizeof(speed));
		dropoff_packet[27] = crc_arr[0];
		dropoff_packet[28] = crc_arr[1];

		int sentisize = -1;
		sentisize = send(sock_, dropoff_packet, sizeof(dropoff_packet), 0);
		ROS_WARN("Sentisize is %d",sentisize);
		ROS_WARN("drop off packet size (should be 29) is %d", (int)sizeof(dropoff_packet));

		if (sentisize > -1) {
			ROS_WARN("SEND ARRIVE AT DROPOFF LOCATION PACKET SUCCESSFULLY");
			sequence_++;
			mission_state = listening;
			return true;
		}
		else {
			ROS_ERROR("FAILED TO SEND ARRIVE AT DROPOFF LOCATION PACKET");
			return false;
		}
	}

	void sendCurrentLocation(double a_latitude, double a_longitude, float a_elevation, unsigned short a_heading, unsigned char a_speed) {

		double latitude = a_latitude;
		double longitude = a_longitude;
		float elevation = a_elevation;
		unsigned short heading = a_heading;
		unsigned char speed = a_speed;

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
			sentisize = send(sock_, location_packet, sizeof(location_packet), 0);
			ROS_WARN("Sentisize is %d",sentisize);
			ROS_WARN("location packet size (should be 34) is %d",(int)sizeof(location_packet));

			if (sentisize  > -1) {
				ROS_WARN("SEND ACCESS LOCATION PACKET SUCCESSFULLY");
				sequence_location++;
			}
			else {
				ROS_ERROR("FAILED TO SEND LOCATION PACKET, PLEASE CHECK CONNECTION STATUS");
			}

	}

	void mapCallback(const path_msgs::Map &msg){
		offset_x = msg.OffsetMapX;
		offset_y = msg.OffsetMapY;
		ROS_INFO("offset_x is %f and offset_y is %f",offset_x,offset_y);
		mapinfo_avail = true;
	}

	// void globalpathCallback(const path_msgs::Trajectory &traj_msg){
	// 	is_path_generated = true;
	// }

	void posealertCallback(const novatel_gps_msgs::InspvaPtr& gpsHandler){
		// update current position
		std:: mutex mtx;
		mtx.lock();
			curr_pos_lat = gpsHandler->latitude;
			curr_pos_long = gpsHandler->longitude;
			curr_pos_z = gpsHandler->height;
			send_heading = (unsigned short) gpsHandler->azimuth;
			north_vel = gpsHandler->north_velocity;
			east_vel = gpsHandler->east_velocity;
			send_speed = (unsigned char) sqrt( east_vel*east_vel + north_vel*north_vel )*3.6; // m/s to km/h conversion
		mtx.unlock();

		double t1 = ros::Time::now().toSec();
		static double t2 = t1;
		double timediff = t1-t2;

		if ((is_connect_ != -1) && (timediff > 0.1)){
			ROS_WARN("[loop time (100ms) : %.4f ] STARTING SEND CURRENT LOCATION THREAD!", timediff*1000);
			std::thread th1(&Call_List_Client::sendCurrentLocation, this, curr_pos_lat, curr_pos_long, curr_pos_z, send_heading, send_speed);
			th1.join();
			t2 = t1;
		} //OFFLINE이면 v2x통신으로 현재 위치를 보내줄 필요가 없음

	}

	void missionCallback(const std_msgs::Float64ConstPtr &speed){
		// ROS_INFO("canbus speed is %f",*speed);
		double curr_speed_time = ros::Time::now().toSec();
		static double prev_speed_time = curr_speed_time;

		std::mutex mtx;
		mtx.lock();
			curr_speed = (float)speed->data;
			if (curr_speed_time - prev_speed_time > 0.05){
				speed_buf_.push_back((float)speed->data);
				if (speed_buf_.size() > 200){
					speed_buf_.erase(speed_buf_.begin(),speed_buf_.begin()+100);
				}
				prev_speed_time = curr_speed_time;

			}
		mtx.unlock();

		///// IF CONNECTED TO SOCKET
		if (is_connect_ != -1){

			double t = ros::Time::now().toSec();
			receive_message();
			t = ros::Time::now().toSec() - t;
			ROS_INFO("TIME TAKEN TO RECEIVE MESSAGE IS : %.5f",t);

			switch (mission_state){
				case listening:{
					ROS_INFO(">>>>MissionState: listening");
					break;
				}
				case no_more_missions:{
					ROS_INFO(">>>>MissionState: no more missions");
					ROS_INFO("\n\n\nALL MISSIONS COMPLETED");
					ros::shutdown();
					break;
				}
				case choosing_mission:{
					ROS_INFO(">>>>MissionState: choosing Mission");

					if (mission_alternative == 0){
						// option 1: 직접 미션 고르기
						mission_in_order = PrioritizeMission("me");
						// option 2: 계산해서 미션 고르기 -> 창기형 node에서 받기 (아직 구현 안됨)
						// std::vector<int> mission_in_order = PrioritizeMission("algorithm");
					} else if (mission_alternative == launchfile_mission_list.size()){ //// YAML쓸때만 해당되는 코드???
						mission_state = no_more_missions;
						break;
					}

					selected_mission = mission_in_order[mission_alternative];

					ROS_ERROR_STREAM("Going to request for mission "<<selected_mission);
					CallRequestMessage(selected_mission);

					break;
				}
				case start_mission:{
					ROS_ERROR("\n\t\t\tCHOSEN MISSION IS %d",selected_mission);
					is_during_mission_ = true;
					showPickDropInRviz();
					ROS_INFO(">>>>MissionState: STARTING MISSION");
					MakeGlobalPath("pickup");
					mission_state = going_to_start_pos;

					break;
				}
				case going_to_start_pos:{
					ROS_INFO(">>>>MissionState: Going to Pick Up Location");
					bool picked = checkArrival("pickup");
					if (picked == true) mission_state = pickingup;
					break;
				}
				case pickingup:{
					ROS_INFO(">>>>MissionState: Picking Up Passenger");
					count5secs();
					if (count_started == false){ //이쪽까지 오면 5초 세는 것이 완료된것임
						StartingPointArriveMessage(selected_mission,curr_pos_lat, curr_pos_long,send_speed);
					}
					break;
				}
				case start_dropoff:{
					ROS_INFO(">>>>MissionState: Starting Drop off mission");
					MakeGlobalPath("dropoff");
					mission_state = going_to_end_pos;
					break;
				}
				case going_to_end_pos:{
					ROS_INFO(">>>>MissionState: Going to Drop Off Location");
					bool dropped = checkArrival("dropoff");
					if (dropped == true) mission_state = droppingoff;
					break;
				}
				case droppingoff:{
					ROS_INFO(">>>>MissionState: Dropping Off Passenger");
					count5secs();
					if (count_started == false){ //이쪽까지 오면 5초 세는 것이 완료된것임
						EndPointArriveMessage(selected_mission,curr_pos_lat, curr_pos_long,send_speed);
					}
					break;
				}
				default:{
					ROS_ERROR("weird case detected! change code\n going to sleep for 100 seconds and then shutdown");
					ros::shutdown();
				}
			}
		}
	}

	void showPickDropInRviz(){

		//rviz에 픽업, 드랍오프 장소 띄우기
		geometry_msgs::PoseStamped rviz;
		rviz.header.frame_id = "map";
		rviz.header.stamp = ros::Time::now();
		rviz.pose.orientation.w = 1;

		// setting pickup position
		std::vector<double> rvizbuf = gpsConv(mission_buf_[selected_mission-1].start_xpos, mission_buf_[selected_mission-1].start_ypos, "LL", "OUR");
		rviz.pose.position.x = rvizbuf[0];
		rviz.pose.position.y = rvizbuf[1];
		pickup_pub_.publish(rviz);

		// setting dropoff position
		rvizbuf = gpsConv(mission_buf_[selected_mission-1].goal_xpos, mission_buf_[selected_mission-1].goal_ypos, "LL", "OUR");
		rviz.pose.position.x = rvizbuf[0];
		rviz.pose.position.y = rvizbuf[1];
		dropoff_pub_.publish(rviz);
	}

	std::vector<int> PrioritizeMission(std::string choosingsubject) {
		std::vector<int> mission_order;
		if (choosingsubject == "me"){
			return launchfile_mission_list;
		} else if (choosingsubject == "algorithm"){
			
			auto tmp_mission_list = mission_buf_;

			// // Cost로 우선순위화하기 -> 창기형이랑 메시지 주고받을 것임
			// 	// DEBUGGING 용:
			// 	for (int i=0; i<tmp_mission_list.size();i++){
			// 		std::cout<<"tmp_mission_list "<<i<<" is id: "<<tmp_mission_list[i].id << " with cost of "<<tmp_mission_list[i].my_priority_score<<std::endl;
			// 	}
			// std::sort(tmp_mission_list.begin(), tmp_mission_list.end(), less_than_key());
			// 	// DEBUGGING 용:
			// 	for (int i=0; i<tmp_mission_list.size();i++){
			// 		std::cout<<"tmp_mission_list "<<i<<" is id: "<<tmp_mission_list[i].id << " with cost of "<<tmp_mission_list[i].my_priority_score<<std::endl;
			// 	}				

			// done_mission_list에 없는 한, mission_order에 추가하기
			for (int i = 0; i < 60; i++){
				// https://stackoverflow.com/questions/3450860/check-if-a-stdvector-contains-a-certain-object
				if (std::find(done_mission_list.begin(),done_mission_list.end(),tmp_mission_list[i].id) == done_mission_list.end()){
					mission_order.push_back(tmp_mission_list[i].id);
				}
			}

		} else {
			ROS_ERROR("|%s| cannot be a subject that chooses the missions", choosingsubject.c_str());
			ros::shutdown();
		}

		return mission_order;
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
		} else {
			ROS_ERROR("WRONG INPUT CONVERSION SETTING| FROM %s to %s DOESNT EXIST",source.c_str(), dest.c_str());
			ros::shutdown();
		}

		positionxy.push_back(x);
		positionxy.push_back(y);

		// ROS_INFO("To %s(x,y) = (%f,%f)",dest.c_str(),positionxy[0],positionxy[1]);

		return positionxy;
	}

	bool checkArrival(const std::string pickup_or_dropoff){ //pickup_or_dropoff = "pickup" OR "dropoff"

		double x = curr_pos_lat;
		double y = curr_pos_long;
		float v = std::accumulate(speed_buf_.end()-50,speed_buf_.end(),0) / 50;
		double dist;

		if (pickup_or_dropoff == "pickup") dist = lindist(mission_buf_[selected_mission-1].start_xpos, mission_buf_[selected_mission-1].start_ypos, x, y )*100000;
		else if (pickup_or_dropoff == "dropoff") dist = lindist(mission_buf_[selected_mission-1].goal_xpos, mission_buf_[selected_mission-1].goal_ypos, x, y )*100000;
		else {
			ROS_ERROR("MENTION IF THIS IS A PICKUP SEQUENCE OR DROPOFF SEQUENCE");
			sleep(100);
			ros::shutdown();
		}

		ROS_INFO("APPROACHING %s POINT, %.2f METERS(?) LEFT| average speed is %.2f",pickup_or_dropoff.c_str(), dist, v);

		if 		(dist < 14.0 && v == 0 && pickup_or_dropoff == "pickup") return true;
		else if (dist < 14.0 && v == 0 && pickup_or_dropoff == "dropoff") return true;

		return false;
	}

	void count5secs (){

		if (count_started == false){
			count_started = true;
			ROS_INFO("counting 5 seconds");
			count5start = ros::Time::now().toSec();
		} else if (count_started == true){
			double count5now = ros::Time::now().toSec();
			double time_diff = count5now - count5start;
			ROS_INFO("waited for %f seconds",time_diff);
			if (time_diff > 5){
				ROS_WARN(" DONE PICKING UP / DROPPING OFF PASSENGER");
				count_started = false;
			}
		}
	}

	void MakeGlobalPath(const std::string pickup_or_dropoff){
		// STEP 1: Current 위치 설정하기
		geometry_msgs::PoseWithCovarianceStamped start_pose_msg;

		std::vector<double> current_position = gpsConv(curr_pos_lat, curr_pos_long,"LL","OUR");
		start_pose_msg.pose.pose.position.x = current_position[0];
		start_pose_msg.pose.pose.position.y = current_position[1];
		start_pose_msg.pose.pose.orientation.x = 0;
		start_pose_msg.pose.pose.orientation.y = 0;
		start_pose_msg.pose.pose.orientation.z = 0;
		start_pose_msg.pose.pose.orientation.w = 1;
		currpose_pub_.publish(start_pose_msg);

		// STEP 2: 현재의 Global Planning 목표 설정하기
		geometry_msgs::PoseStamped target_msg;
		target_msg.header.stamp = ros::Time::now();
		target_msg.header.frame_id = "map";
		target_msg.pose.orientation.x = 0;
		target_msg.pose.orientation.y = 0;
		target_msg.pose.orientation.z = 0;
		target_msg.pose.orientation.w = 1;


		if (pickup_or_dropoff == "pickup"){
			std::vector<double> position_xy = gpsConv( mission_buf_[selected_mission-1].start_xpos, mission_buf_[selected_mission-1].start_ypos , "LL", "OUR");
			ROS_INFO("pickup position goal is (%f,%f)",position_xy[0],position_xy[1]);
			target_msg.pose.position.x = position_xy[0];
			target_msg.pose.position.y = position_xy[1];
			target_pub_.publish(target_msg);
			ROS_INFO("GLOBAL PATH TO PICKUP PUBLISHED");
			ROS_INFO("x,y = %f,%f",position_xy[0],position_xy[1]);

		} else if (pickup_or_dropoff == "dropoff"){
			std::vector<double> position_xy = gpsConv( mission_buf_[selected_mission-1].goal_xpos, mission_buf_[selected_mission-1].goal_ypos, "LL", "OUR");
			ROS_INFO("dropoff position goal is (%f,%f)",position_xy[0],position_xy[1]);
			target_msg.pose.position.x = position_xy[0];
			target_msg.pose.position.y = position_xy[1];
			target_pub_.publish(target_msg);
			ROS_INFO("GLOBAL PATH TO DROP OFF PUBLISHED");

		} else {
			ROS_ERROR("MENTION IF THIS IS A PICKUP SEQUENCE OR DROPOFF SEQUENCE");
			ros::shutdown();
		}

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

	std::string static convertArrToHexStr(char *data, int len) {
		std::string s(len * 2, ' ');
		for (int i=0;i<len;++i) {
			s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
			s[2 * i + 1] = hexmap[data[i] & 0x0F];
		}
		return s;
	}

	void save_payload(std::string payload, std::string comment){
	//https://stackoverflow.com/questions/3381614/c-convert-string-to-hexadecimal-and-vice-versa
		std::string output = string_to_hex(payload);
		std::cout<<output<<std::endl;
		std::ofstream myfile;
		myfile.open("/home/urlautocar/v2x_payloads.txt");
		myfile << comment;
		myfile << output;
		myfile.close();
	}

	void save_payload(char * payload, std::string comment){
		std::string output = convertArrToHexStr(payload, (int) sizeof(*payload));
		std::cout<<output<<std::endl;
		std::ofstream myfile;
		myfile.open("/home/urlautocar/v2x_payloads.txt");
		myfile << comment;
		myfile << output;
		myfile.close();
	}


};

int main(int argc, char** argv) {

	ros::init(argc, argv, "call_list_client_node");

	Call_List_Client clc;

	ros::spin();

	return 0;
}
