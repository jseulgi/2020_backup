#ifndef CANCLASS_H
#define CANCLASS_H
#include "ros/ros.h"
#include "control_msgs/CanFrame.h"
#include "control_msgs/Vehicle_cmd.h"
#include "control_msgs/Vehicle_status.h"
#include "control_msgs/VehicleState.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

class CAN
{
public:
    class EpsParsing
    {
    public:
        uint8_t EPS_Fd_En;
        uint8_t Mo_Fd_State_EPS;
        uint8_t Mo_Fd_Seq_EPS;
        uint8_t EPS_Fd_ErrBit_pd;
        uint8_t EPS_Fd_ErrBit_EPS;
        uint8_t EPS_Fd_ErrBit_Vinfo;
        uint8_t EPS_Fd_ErrBit_SAS;
        uint8_t EPS_Fd_Ovrd_Ignore;
        uint8_t EPS_Fd_Override;
        double EPS_Fd_SteerAngle;
        double EPS_Fd_Steer_Torq;
        double EPS_Fd_Str_out_tq;
        uint8_t Mo_Fd_AlvCnt_EPS;
    };

    class AccParsing
    {
    public:
        uint8_t ACC_Fd_En;
        uint8_t Mo_Fd_State;
        uint8_t Mo_Fd_Seq_ACC;
        uint8_t ACC_Fd_ErrBit_PD;
        uint8_t ACC_Fd_ErrBit_TCU;
        uint8_t ACC_Fd_ErrBit_ACC;
        uint8_t ACC_Fd_aeb_seq;
        uint8_t ACC_Fd_VSpeed;
        uint8_t ACC_Disp_SetSpeed;
        double LONG_ACCEL_ACC;
        uint8_t ACC_Fd_Ovrrd_Rturn_sig;
        uint8_t ACC_Fd_Ovrrd_haz_sig;
        uint8_t ACC_Fd_Ovrrd_Lturn_sig;
        uint8_t G_SEL_DISP;
        uint8_t Mo_Fd_Alv_Cnt_ACC;
    };

    class WheelParsing
    {
    public:
        double WHL_SPD_FL;
        double WHL_SPD_FR;
        double WHL_SPD_RL;
        double WHL_SPD_RR;
    };

    class InsParsing
    {
    public:
        double LONG_ACCEL_INS;
        double YAW_RATE;
        uint8_t Mo_Fd_Alv_Cnt_INS;
    };

    class MoConf
    {
    public:
        uint8_t EPS_En;
        uint8_t EPS_IGNORE;
        double EPS_Slvel;
        uint8_t ACC_En;
        uint8_t ACC_StopRequest;
        uint8_t ACC_aeb_seq;
        uint8_t SetDispSpeed;
        uint8_t Mo_AlvCnt;

        MoConf(){
          EPS_En = EPS_IGNORE = ACC_En = ACC_StopRequest = 0;
          ACC_aeb_seq = SetDispSpeed = Mo_AlvCnt = 0;
          EPS_Slvel = 0.0;
        }
    };

    class MoVal
    {
    public:
        double Steer_Cmd;
        double Accel_Dec_Cmd;
        uint8_t Aeb_DecCmd;
    };

    int flag;

    //Cammand For Publish Topic to Vehicle

    CAN::EpsParsing GetEpsData(){ return eps; }
    CAN::AccParsing GetAccData(){ return acc; }
    CAN::WheelParsing GetWheelData(){ return wheel; }
    CAN::InsParsing GetInsData(){ return ins; }
    CAN::MoConf GetMoConfData(){ return mo_conf; }
    CAN::MoVal GetMoValData(){ return mo_val; }

    CAN();
    ~CAN();

private:

    ros::NodeHandle m_nh;

    // ErrorBit Topics
    std_msgs::UInt8 EpsErrorPd;
    std_msgs::UInt8 EpsErrorEps;
    std_msgs::UInt8 EpsErrorVinfo;
    std_msgs::UInt8 EpsErrorSas;
    std_msgs::UInt8 AccErrorTcu;
    std_msgs::UInt8 AccErrorAcc;

    // These Parameters are set in the Functions about the Conf.
    std_msgs::UInt8 AllEnableStatus;
    std_msgs::UInt8 EstopStatus;
    std_msgs::UInt8 EpsEnableStatus;
    std_msgs::UInt8 AccEnableStatus;
    std_msgs::UInt8 EpsIgnoreStatus;

    //Digit Text
    std_msgs::String SteerCmdShow;
    std_msgs::String AccelCmdShow;
    std_msgs::String StatusStringShow;

    control_msgs::CanFrame PubCmd156;
    control_msgs::CanFrame PubCmd157;

    //Publisher
    ros::Publisher eps_fd_errbit_pd_pub;
    ros::Publisher eps_fd_errbit_eps_pub;
    ros::Publisher eps_fd_errbit_vinfo_pub;
    ros::Publisher eps_fd_errbit_sas_pub;
    ros::Publisher acc_fd_errbit_tcu_pub;
    ros::Publisher acc_fd_errbit_acc_pub;

    ros::Publisher all_auto_status_pub;
    ros::Publisher estop_status_pub;
    ros::Publisher steering_status_pub;
    ros::Publisher acc_status_pub;
    ros::Publisher steering_override_status_pub;
    ros::Publisher vehicle_state_pub;


    //DigitText
    ros::Publisher steer_cmd;
    ros::Publisher accel_cmd;
   // ros::Publisher status_string;
    ros::Publisher pub_speed;

    ros::Publisher Cmd156_pub;
    ros::Publisher Cmd157_pub;


    //Initializer Function
    void Initializer();

    //Data Parsing
    void EPS_Parsing(vector<uint8_t> Data);
    void ACC_Parsing(vector<uint8_t> Data);
    void Wheel_Parsing(vector<uint8_t> Data);
    void INS_Parsing(vector<uint8_t> Data);

    // Service -> Enable Check Function
    void ConfAllEnableStatus();
    void ConfSteerEnableStatus();
    void ConfAccEnableStatus();
    void ConfEstopEnableStatus();
    void ConfSteerIgnoreStatus();

    //FeedBack Errorbit Check
    void EPSFdErrorCheck();
    void AccFdErrorCheck();
    void ModeCheck(); //MoFdState and MoFdSeq

    //PID Controller Function
    double PIDController(double target, double current);
    float PIDSaturate(double input, float min, float max);

    // Get Current Status, Velocity, Steer Angle.
    void DigitText();

    // Status Topics for Vehicle Mode
    void VehicleStatus();

    // Publish Command
    void PubCmd();

    //Subscriber
    void CanCallback(const control_msgs::CanFrame::ConstPtr& Can);
    void AccelerationCallback(const std_msgs::Float64::ConstPtr& Acceleration);
    void TargetVelocityCallback(const std_msgs::Float64::ConstPtr& TargetVelocity);
    void SteerCmdCallback(const control_msgs::Vehicle_cmd::ConstPtr& Steering);

    //Service
    bool all_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res);
    bool steer_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res);
    bool acc_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res);
    bool estop_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res);
    bool steer_ignore(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res);

    //Function to Publish Topics
    void PublishTopics(); //const ros::TimerEvent&

    CAN::EpsParsing eps;
    CAN::AccParsing acc;
    CAN::WheelParsing wheel;
    CAN::InsParsing ins;
    CAN::MoConf mo_conf;
    CAN::MoVal mo_val;
    control_msgs::VehicleState vehicle_state;


};

#endif // CAN_CLASS_H
