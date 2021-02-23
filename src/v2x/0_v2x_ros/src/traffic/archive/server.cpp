// ROS
#include <ros/ros.h>
#include <v2x_ros/PVD.h>
#include <v2x_ros/BSM.h>

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

#include "obu_message.h"
#include "sample_message.h"

#include "MessageFrame.h"
#include "dsrc_msg_id.h"

// PVD PARAMETER
#define SIZE_ENCODE_BUF_MAX 2048
#define SIZE_PVD_ID_BUF 4
#define NUM_PVD_SNAPSHOTS 10

bool is_done_ = false;

class V2X_Server {
public:	
	V2X_Server(void):
		private_nh_("~")	
	{
		initParams();
		initSetup();	
		ROS_INFO("V2X SERVER NODE INITIALIZED");
	}

	void initParams(void) {
		private_nh_.getParam("obu_port", obu_port_);
		private_nh_.getParam("optname", optname_);
		count_ = 0;
		len_addr_ = sizeof(obu_addr_);
		sock_ = -1;
		new_sock_ = -1;
	}
	void initSetup(void)
	{
		initSignalHandler();

		if ((sock_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
			ROS_ERROR("FAILED TO CREATE SOCKET FILE DESCRIPTOR");
			ros::shutdown();
		}

		if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &optname_, sizeof(optname_))) {
			ROS_ERROR("FAILED TO SET THE SOCKET OPTION");
		       	ros::shutdown();	
		}

		obu_addr_.sin_family = AF_INET;
		obu_addr_.sin_addr.s_addr = INADDR_ANY;
		obu_addr_.sin_port = htons(obu_port_);

		if (bind(sock_, (struct sockaddr *)&obu_addr_, sizeof(obu_addr_)) < 0) {
			ROS_ERROR("FAILED TO BIND THE SOCKET TO THE PORT [%d]", obu_port_);
			ros::shutdown();
		}	
		
		pvd_sub_ = nh_.subscribe("pvd", 10, &V2X_Server::pvdCallback, this);
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
	void pvdCallback(const v2x_ros::PVDConstPtr &pvd_msg) {
		if (listen(sock_, 3) < 0) {
			ROS_ERROR("FAILED TO CREATE CONNECTION QUEUE");
			return;
		}
		if ((new_sock_ = accept(sock_, (struct sockaddr *)&obu_addr_, (socklen_t*)&len_addr_)) < 0) {
			ROS_ERROR("FAILED TO ACCEPT THE CONNECTION FROM SOCKET");
			return;
		}
	    
		std::string payload;

		MessageFrame_t *msg = (MessageFrame_t *)calloc(1, sizeof(MessageFrame_t)); 
	    	msg->messageId = dsrc_msg_id::PROBE_VEHICLE_DATA;
	    	msg->value.present = MessageFrame__value_PR::MessageFrame__value_PR_ProbeVehicleData;
	    
	    	auto &pvd = msg->value.choice.ProbeVehicleData;	
	   
	  	pvd.probeID = new VehicleIdent{0, };
	    	pvd.probeID->name = new DescriptiveName_t{0, };
	    	std::string temp_name = "cest";
	    	pvd.probeID->name->size = temp_name.size();
	    	pvd.probeID->name->buf = new uint8_t[(int)temp_name.size()] {0, };
	    	sprintf((char*)pvd.probeID->name->buf, "%s", temp_name.c_str());
	    
	   	pvd.probeID->id = (VehicleID *)calloc(1, sizeof(VehicleID));
	    	pvd.probeID->id->present = VehicleID_PR::VehicleID_PR_entityID;
	   	pvd.probeID->id->choice.entityID.size = SIZE_PVD_ID_BUF;
	    	pvd.probeID->id->choice.entityID.buf = new uint8_t[SIZE_PVD_ID_BUF]{0x12, 0x34, 0x56, 0x78};

	    	pvd.startVector.utcTime = new DDateTime{0, };
	    	pvd.startVector.utcTime->year = new DYear_t(2020);
	    	pvd.startVector.utcTime->month = new DMonth_t(7);
	    	pvd.startVector.utcTime->day = new DDay_t(22);
	    	pvd.startVector.utcTime->hour = new DHour_t(14);
	    	pvd.startVector.utcTime->minute = new DMinute_t(58);
	    	pvd.startVector.utcTime->second = new DSecond_t(34000);
	    	pvd.startVector.utcTime->offset = new DOffset_t(0);	

	    	pvd.startVector.Long = 1286148415;
	    	pvd.startVector.lat = 358920922;
	    	pvd.startVector.elevation = new DSRC_Elevation_t(533);
	    	pvd.startVector.heading = new Heading_t(26556);
	    	pvd.startVector.speed = new TransmissionAndSpeed{0, };
	    	pvd.startVector.speed->transmisson = TransmissionState::TransmissionState_unavailable;
	    	pvd.startVector.speed->speed = 600;

	    	pvd.startVector.posAccuracy = new PositionalAccuracy{0, };
	    	pvd.startVector.posAccuracy->semiMajor = 11;
	    	pvd.startVector.posAccuracy->semiMinor = 22;
	    	pvd.startVector.posAccuracy->orientation = 33;

	    	pvd.vehicleType.vehicleType = new VehicleGroupAffected_t(VehicleGroupAffected::VehicleGroupAffected_cars);
	    	pvd.vehicleType.keyType = new BasicVehicleClass_t(0);
	    
	   	pvd.snapshots.list.count = NUM_PVD_SNAPSHOTS;
	    	pvd.snapshots.list.size = sizeof(Snapshot) * NUM_PVD_SNAPSHOTS;
	    	pvd.snapshots.list.array = (Snapshot **)calloc(NUM_PVD_SNAPSHOTS, sizeof(Snapshot*));
	    	for(int i=0;i<NUM_PVD_SNAPSHOTS;i++) {
			pvd.snapshots.list.array[i] = (Snapshot *)calloc(1, sizeof(Snapshot)); 
			auto &thePosition = pvd.snapshots.list.array[i]->thePosition;
		
			thePosition.utcTime = new DDateTime{0, };
			thePosition.utcTime->year = new DYear_t(2020);
			thePosition.utcTime->month = new DMonth_t(7);
			thePosition.utcTime->day = new DDay_t(22);
			thePosition.utcTime->hour = new DHour_t(14);
			thePosition.utcTime->minute = new DMinute_t(58);
			thePosition.utcTime->second = new DSecond_t(34000 + i * 100);
			thePosition.utcTime->offset = new DOffset_t(0);	

			thePosition.Long = 1286148415 + i * 100;
			thePosition.lat = 358920922 + i * 100;
			thePosition.elevation = new DSRC_Elevation_t(533 + i * 10);
			thePosition.heading = new Heading_t(26556 - i * 100);
			thePosition.speed = new TransmissionAndSpeed{0, };
			thePosition.speed->transmisson = TransmissionState::TransmissionState_unavailable;
			thePosition.speed->speed = 600 + i * 10;
	
			thePosition.posAccuracy =new PositionalAccuracy{0, };
			thePosition.posAccuracy->semiMajor = 11 + i;
			thePosition.posAccuracy->semiMinor = 22 + i;
			thePosition.posAccuracy->orientation = 33 + i;        
		}
	    
	    	uint8_t buffer[SIZE_ENCODE_BUF_MAX] = {0, };
		asn_enc_rval_t er;
		er = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, msg, buffer, SIZE_ENCODE_BUF_MAX);
	    
	   	if (er.encoded == -1) {
			ROS_INFO("FAILED TO ENCODE PVD PAYLOAD");
	    	}
	    	else {
			int size = ceil((double)er.encoded / 8.);
			if (size < SIZE_ENCODE_BUF_MAX) {
		    		payload.append((char*)buffer, size);
		    		ROS_INFO("ENCODED PVD PAYLOAD SUCCESSFULLY, [%d Bytes]", size);
		    
		    		uint8_t *temp =  (uint8_t*)payload.c_str();
			}
			else {
		    		ROS_INFO("EXCEEDED [%d Bytes], PLEASE INCREASE THE MAXIMUM BUFFER SIZE", size - SIZE_ENCODE_BUF_MAX);
			}
	    	}
	   	ASN_STRUCT_FREE(asn_DEF_MessageFrame, msg);

		obu_tcp_header_t header;
		header.packet_type = 0x3412;
		header.current_sequence = count_;
		header.payload_size = (uint16_t)2;
		header.device_id[0] = (uint8_t)7;
		header.device_id[1] = (uint8_t)7;
		header.device_id[2] = (uint8_t)7;

		unsigned char payload_arr[sizeof(payload)];
		std::copy(payload.begin(), payload.end(), payload_arr);

		unsigned char *header_arr = (unsigned char*)malloc(sizeof(header));
		memcpy(header_arr, (const unsigned char*)&header, sizeof(header));
	
		unsigned char *msg_arr = (unsigned char*)malloc(sizeof(header) + sizeof(payload_arr));
		
		memcpy(msg_arr, (const unsigned char*)header_arr, sizeof(header));
		memcpy(msg_arr + sizeof(header) / sizeof(unsigned char), (const unsigned char*)payload_arr, sizeof(payload_arr));
	
		int sentisize = -1;
		
		if (sentisize = send(new_sock_, msg_arr, sizeof(header) + sizeof(payload_arr), 0) > 0) {
			ROS_INFO("SENT PVD MESSAGE SUCESSFULLY, [%d Bytes]", sentisize);
		}

		count_++;	
	}
private:
	ros::NodeHandle private_nh_;
	ros::NodeHandle nh_;
	ros::Subscriber pvd_sub_;

	sockaddr_in obu_addr_;
	int len_addr_, obu_port_, sock_, new_sock_, optname_, count_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "v2x_server_node");

	V2X_Server vs;

	return 0;
}
