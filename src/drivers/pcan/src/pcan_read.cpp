#include <ros/ros.h>
#include <stdint.h>

#include "pcan/PCANBasic.h"
#include "autonomous_car/CanFrame.h"

#define PCAN_DEVICE PCAN_PCIBUS2

using namespace std;

typedef uint8_t uint8;
typedef uint32_t uint32;

int main(int argc, char* argv[])
{
        ros::init(argc, argv, "pcan_read");
        ros::NodeHandle nh;

        ros::Publisher can_pub = nh.advertise<control_msgs::CanFrame>("/can_bus1", 1);

        TPCANMsg Message;
        TPCANStatus Status;

        Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);
        printf("Initialize CAN2 : %i\n",(int)Status);

        while (ros::ok()) {
       	    while ((Status = CAN_Read(PCAN_DEVICE,&Message,NULL)) == PCAN_ERROR_QRCVEMPTY)
                usleep(10);

            if (Status != PCAN_ERROR_OK) {
                printf("CAN2 Error 0x%x\n",(int)Status);
                break;
            }
               
            // control_msgs::CanFrame can;
            // can.ID = (uint32)Message.ID;
            // can.timestamp = ros::Time::now();
            // can.Length = (uint8)Message.LEN;
            // for(int i = 0; i < Message.LEN; i++) can.Data.push_back((uint8)Message.DATA[i]);
            // can_pub.publish(can);

            if (((uint32)Message.ID >= 0x710 && (uint32)Message.ID <= 0x713) || (uint32)Message.ID == 0x420 ){
                control_msgs::CanFrame can;
                can.ID = (uint32)Message.ID;
                can.timestamp = ros::Time::now();
                can.Length = (uint8)Message.LEN;
                for(int i = 0; i < Message.LEN; i++) can.Data.push_back((uint8)Message.DATA[i]);
                can_pub.publish(can);
            }

            ros::spinOnce();
        }

        return 0;
}
