#include <ros/ros.h>
#include <stdint.h>
#include <vector>

#include "pcan/PCANBasic.h"
#include "control_msgs/CanFrame.h"


// #define PCAN_DEVICE     PCAN_USBBUS1
#define PCAN_DEVICE     PCAN_PCIBUS1

using namespace std;

typedef uint8_t uint8;
typedef uint32_t uint32;

bool is_initializaed_can = false;
vector<control_msgs::CanFrame> can_msg_buf;
ros::Publisher can_pub;

void CallbackVehicleCMD(const control_msgs::CanFrame::ConstPtr& can){
    if (!is_initializaed_can) return;

    TPCANMsg Message;

    Message.ID = can->ID;
    Message.LEN = can->Length;
    Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    for(int i = 0; i < Message.LEN; i++) Message.DATA[i] = can->Data[i];

    CAN_Write(PCAN_DEVICE,&Message);
    ROS_WARN("PCAN: %f", ros::Time::now().toSec());
    printf("PCAN: %f\n", ros::Time::now().toSec());
    ROS_WARN("3: %x", Message.DATA[3]);
    printf("3: %x\n", Message.DATA[3]);
    ROS_WARN("4: %x", Message.DATA[4]);
    printf("4: %x\n", Message.DATA[4]);
}

void CallbackVehicleConf(const control_msgs::CanFrame::ConstPtr& can){
    if (!is_initializaed_can) return;

    TPCANMsg Message;

    Message.ID = can->ID;
    Message.LEN = can->Length;
    Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    for(int i = 0; i < Message.LEN; i++) Message.DATA[i] = can->Data[i];

    CAN_Write(PCAN_DEVICE,&Message);
}

void timerCallback(const ros::TimerEvent&){
    if (can_msg_buf.size() < 1 || !is_initializaed_can) return;

    can_pub.publish(can_msg_buf[0]);
    can_msg_buf.erase(can_msg_buf.begin());
}

int main(int argc, char* argv[])
{
        ros::init(argc, argv, "pcan_read_write");
        ros::NodeHandle nh;

        can_pub = nh.advertise<control_msgs::CanFrame>("/can_bus1", 1);

        ros::Subscriber can_vehicle_cmd  = nh.subscribe("/can_write_vehicle_cmd", 10, CallbackVehicleCMD);
        ros::Subscriber can_vehicle_conf = nh.subscribe("/can_write_vehicle_conf", 10, CallbackVehicleConf);

        ros::Timer timer = nh.createTimer(ros::Duration(0.001), timerCallback);

        TPCANMsg Message;
        TPCANStatus Status;

        Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);
        printf("Initialize CAN1 : %i\n",(int)Status);
        is_initializaed_can = (Status == PCAN_ERROR_OK ? true : false) ;

        while (ros::ok()) {
            while ((Status = CAN_Read(PCAN_DEVICE,&Message,NULL)) == PCAN_ERROR_QRCVEMPTY)
                if(usleep(10))
                    break;

            if (Status != PCAN_ERROR_OK) {
                printf("CAN1 Error 0x%x\n",(int)Status);
                break;
            }

            if (((uint32)Message.ID >= 0x710 && (uint32)Message.ID <= 0x713) || (uint32)Message.ID == 0x420 ){
                control_msgs::CanFrame can;
                can.ID = (uint32)Message.ID;
                can.timestamp = ros::Time::now();
                can.Length = (uint8)Message.LEN;
                for(int i = 0; i < Message.LEN; i++) can.Data.push_back((uint8)Message.DATA[i]);
                can_msg_buf.push_back(can);
            }

            ros::spinOnce();
        }

        CAN_Uninitialize(PCAN_DEVICE);

        return 0;
}
