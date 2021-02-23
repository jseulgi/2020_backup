#include "CanClass.h"


CAN::CAN()
{
    Initializer();
    static ros::Subscriber can_subscriber = m_nh.subscribe<control_msgs::CanFrame>("can_bus1",1000,&CAN::CanCallback,this);
    static ros::Subscriber TargetVelocity_Subscriber = m_nh.subscribe<std_msgs::Float64>("Target_Velocity",1000,&CAN::TargetVelocityCallback,this);
    static ros::Subscriber Acceleration_Subscriber = m_nh.subscribe<std_msgs::Float64>("Acceleration",1000,&CAN::AccelerationCallback,this);
    static ros::Subscriber Steering_Subscriber = m_nh.subscribe<control_msgs::Vehicle_cmd>("Steering_cmd",1000,&CAN::SteerCmdCallback,this);
    static ros::ServiceServer EpsAccEnableService = m_nh.advertiseService("all_auto_enable",&CAN::all_enable,this);
    static ros::ServiceServer SteeringWheelEnableService = m_nh.advertiseService("steering_wheel_enable",&CAN::steer_enable,this);
    static ros::ServiceServer AccEnableService = m_nh.advertiseService("acc_enable",&CAN::acc_enable,this);
    static ros::ServiceServer EmergencyStopService = m_nh.advertiseService("estop_enable",&CAN::estop_enable,this);
    static ros::ServiceServer EpsIgnoreService = m_nh.advertiseService("steering_override_ignore",&CAN::steer_ignore,this);
    eps_fd_errbit_pd_pub = m_nh.advertise<std_msgs::UInt8>("EPS_ErrBit_PD",10);
    eps_fd_errbit_eps_pub = m_nh.advertise<std_msgs::UInt8>("EPS_ErrBit_EPS",10);
    eps_fd_errbit_vinfo_pub = m_nh.advertise<std_msgs::UInt8>("EPS_ErrBit_VInfo",10);
    eps_fd_errbit_sas_pub = m_nh.advertise<std_msgs::UInt8>("EPS_ErrBit_SAS",10);
    acc_fd_errbit_tcu_pub = m_nh.advertise<std_msgs::UInt8>("ACC_ErrBit_TCU",10);
    acc_fd_errbit_acc_pub = m_nh.advertise<std_msgs::UInt8>("ACC_ErrBit_ACC",10);
    all_auto_status_pub = m_nh.advertise<std_msgs::UInt8>("all_auto_status",10);
    estop_status_pub = m_nh.advertise<std_msgs::UInt8>("emergency_stop_status",10);
    steering_status_pub = m_nh.advertise<std_msgs::UInt8>("/Steering_status",10);
    acc_status_pub = m_nh.advertise<std_msgs::UInt8>("acc_status",10);
    steering_override_status_pub = m_nh.advertise<std_msgs::UInt8>("steering_override_status",10);
    vehicle_state_pub = m_nh.advertise<control_msgs::VehicleState>("/vehicle_state",10);


    // temporary
    pub_speed = m_nh.advertise<std_msgs::Float64>("vehicle_speed",10);

    steer_cmd = m_nh.advertise<std_msgs::String>("Current_Steer_Angle",10); // It is showing the Current Steering Angle Value as String
    accel_cmd = m_nh.advertise<std_msgs::String>("Vehicle_Speed",10);
    // status_string = m_nh.advertise<std_msgs::String>("Status: ",10);

    Cmd156_pub = m_nh.advertise<control_msgs::CanFrame>("can_write_vehicle_conf",10);
    Cmd157_pub = m_nh.advertise<control_msgs::CanFrame>("can_write_vehicle_cmd",10);

    //static ros::Timer timer = m_nh.createTimer(ros::Duration(0.1), &CAN::PublishTopics,this);
}

CAN::~CAN()
{

}

void CAN::Initializer()
{
    mo_conf.EPS_En = 0;
    mo_conf.ACC_En =0;
    mo_conf.ACC_aeb_seq=0;
    mo_conf.ACC_StopRequest=0;
    mo_conf.EPS_IGNORE=0;
    mo_conf.Mo_AlvCnt=0;
    mo_conf.EPS_Slvel=250;
    mo_conf.SetDispSpeed=0;
}


//######################## Data Parsing Functions ###################################################
void CAN::EPS_Parsing(vector<uint8_t> Data)
{
    //std::cout << "EPS EN : " << (int)(Data[0] & 0x01) << std::endl;
    eps.EPS_Fd_En = Data[0] & 0x01;                // EPS_Fd_En: Manual / Auto Mode feedback - EPS_En
    eps.Mo_Fd_State_EPS = (Data[0] & 0x0F)>>1;     // Mo_Fd_State: The Status of the Control Board (0:Abnormal (Check Err bit), 1: Initial State, 2: Normal State)
    eps.EPS_Fd_ErrBit_pd = (Data[0] & 0x10) >>4;   // EPS_Fd_ErrBit_PD: if the data is not comming from CAN for a while, Error is on. Then the AEB is on automatically
    eps.EPS_Fd_ErrBit_EPS = (Data[0] & 0x30) >>5;  // EPS_Fd_ErrBit_EPS: The Error bit of the Control module of the Steering
    eps.EPS_Fd_ErrBit_Vinfo = (Data[0] & 0x70) >>6;// EPS_Fd_ErrBit_VInfo: The Error bit of the Information of car
    eps.EPS_Fd_ErrBit_SAS = (Data[0] & 0xF0) >>7;  // EPS_Fd_ErrBit_SAS: The Error bit of the Sensor of the Steer angle.
    eps.Mo_Fd_Seq_EPS = (Data[1] & 0x0F);          // Mo_Fd_Seq: Mode(0: None, 1: Autonomous Ready, 2:Autonomous All, 3: Autonomous ACC 4: Autonomous Steer, 5-10: Steering Overrided.
    eps.EPS_Fd_Ovrd_Ignore = (Data[1] & 0x10) >>4; // EPS_Fd_Ovrrd_Ignore: Feedback bit of the override ignore.
    eps.EPS_Fd_Override = (Data[1] & 0x30) >>5;    // EPS_Fd_Override: Feedback bit of the override (In the case of the Override, the bit will be '1' for a second.)
    eps.EPS_Fd_SteerAngle = ((short)((Data[3] & 0xFF) <<8) | (Data[2] & 0xFF))*0.1;          // EPS_Fd_SteerAngle: The value of the Steer Angle
    eps.EPS_Fd_Steer_Torq = (((((Data[4] & 0xFF)) | (short)(Data[5] & 0x0F)<<8))-2048)*0.01; // EPS_Fd_Steer_Torq: The value of the Torque of the Steer
    eps.EPS_Fd_Str_out_tq = ((((Data[6] & 0xFF) <<4) | ((Data[5] & 0xF0)))-2048)*0.1;        // EPS_Fd_Str_out_tq: The value of the Out torque of the Steer
    eps.Mo_Fd_AlvCnt_EPS = Data[7];                // Mo_Fd_AlvCnt_EPS: Alivce Count Feedback
}

void CAN::ACC_Parsing(vector<uint8_t> Data)
{
    acc.ACC_Fd_En = Data[0] & 0x01;                   //ACC_Fd_En: Manual / Auto Mode Feedback -ACC_En
    acc.Mo_Fd_State = Data[0] & 0x0F >>1;             //Mo_Fd_State: Same with the EPS
    acc.ACC_Fd_ErrBit_PD = (Data[0] & 0x10) >>4;      //ACC_Fd_ErrBit_PD: Same with the EPS
    acc.ACC_Fd_ErrBit_TCU = (Data[0] & 0x70) >>6;     //ACC_Fd_ErrBit_TCU: Error Bit of the TCU Module
    acc.ACC_Fd_ErrBit_ACC = (Data[0] & 0x80) >>7;     //ACC_Fd_ErrBit_ACC: Error Bit of the controller of the Accelerator
    acc.Mo_Fd_Seq_ACC = (Data[1] & 0x0F);             //Mo_Fd_Seq: Same with the EPS
    acc.ACC_Fd_aeb_seq = (Data[1] & 0x70)>>6;         //ACC_Fd_aeb_seq: Check whether the AEB is working well or not
    acc.ACC_Fd_VSpeed = Data[2];                      //ACC_Fd_VSpeed: Current Vehicle Speed (km/h)
    acc.ACC_Disp_SetSpeed = Data[3];                  //ACC_Fd_SetSpeed: Display the Vehicle Speed
    acc.LONG_ACCEL_ACC = ((((Data[5] & 0x07)<<8) | (Data[4] & 0xFF))-1023)*0.01;  // LONG_ACCEL_ACC: The value of the Acceleration in translational
    acc.ACC_Fd_Ovrrd_Rturn_sig = Data[6] & 0x00;      //ACC_Fd_Ovrrd_Rturn_Sig: Signal of the Right Turn
    acc.ACC_Fd_Ovrrd_haz_sig = (Data[6] & 0x00) >>1;  //ACC_Fd_Ovrrd_Haz_sig: Signal of the Emergency
    acc.ACC_Fd_Ovrrd_Lturn_sig = (Data[6] & 0x00) >>2;//ACC_Fd_Ovrrd_Lturn_Sig: Signal of the Left Turn
    acc.G_SEL_DISP = (Data[6] & 0xF0) >>4;            //G_SEL_DISP: Information of the Gear (P:0x00, R:0x70, N:0x60, D:0x50)
    acc.Mo_Fd_Alv_Cnt_ACC = Data[7];                  //Mo_Fd_AlvCnt_ACC: Alive Count Feedback

}

void CAN::Wheel_Parsing(vector<uint8_t> Data)
{
    wheel.WHL_SPD_FL = (((Data[1] & 0x3F) <<8) | (Data[0]))*0.03125;  //WHL_SPD_FL: Wheel Speed of the Front Left
    wheel.WHL_SPD_FR = (((Data[3] & 0x3F) <<8) | (Data[2]))*0.03125;  //WHL_SPD_FR: Wheel Speed of the Front Right
    wheel.WHL_SPD_RL = (((Data[5] & 0x3F) <<8) | (Data[4]))*0.03125;  //WHL_SPD_RL: Wheel Speed of the Rear Left
    wheel.WHL_SPD_RR = (((Data[7] & 0x3F) <<8) | (Data[6]))*0.03125;  //WHL_SPD_RL: Wheel Speed of the Rear Right
}

void CAN::INS_Parsing(vector<uint8_t> Data)
{
    ins.LONG_ACCEL_INS = ((((Data[1] & 0x07)<<8) | Data[0])-1023)*0.01; // LONG_ACCEL_INS: The Value of the Acceleration in rotational
    ins.YAW_RATE = ((((Data[4] & 0x00FF)<<8) | Data[3]&0x00FF)-4095)*0.01;       // YAW_RATE: yaw rate
    ins.Mo_Fd_Alv_Cnt_INS = Data[7];                                    // Mo_Fd_AlvCnt_INS: Alive Count Feedback
}




//############################### Service Status Check (Used in Service bool~~)######################################
// Service Function, if value is 0 -> value to be 1
void CAN::ConfAllEnableStatus()
{
    if(GetMoConfData().EPS_En ==0 && GetMoConfData().ACC_En==0)
    {
        mo_conf.EPS_En = (uint8_t)1;
        mo_conf.ACC_En = (uint8_t)1;
        // ROS_INFO("AllEnable on\n");
    }
    else if(GetMoConfData().EPS_En ==0 && GetMoConfData().ACC_En==1)
    {
        mo_conf.EPS_En =(uint8_t)1;
        // ROS_INFO("AllEnable on\n");

    }
    else if(GetMoConfData().EPS_En ==1 && GetMoConfData().ACC_En ==0)
    {
        mo_conf.ACC_En = (uint8_t)1;
        // ROS_INFO("AllEnable on\n");
    }
    else {
        mo_conf.EPS_En=(uint8_t)0;
        mo_conf.ACC_En=(uint8_t)0;
        //ROS_INFO("AllEnable off\n");
    }
}
// Service Function, if value is 0 -> value to be 1
void CAN::ConfSteerEnableStatus()
{
    if(GetMoConfData().EPS_En==0)
    {
        mo_conf.EPS_En = (uint8_t)1;
        ROS_INFO("SteerEnable On\n");
    }
    else if(GetMoConfData().EPS_En==1){
        mo_conf.EPS_En = (uint8_t)0;
        ROS_INFO("SteerEnable Off\n");
    }
}
// Service Function, if value is 0 -> value to be 1
void CAN::ConfAccEnableStatus()
{
    if(GetMoConfData().ACC_En ==0)
    {
        mo_conf.ACC_En = (uint8_t)1;
        // ROS_INFO("Acc Enable On\n");
        ROS_INFO("ACC_En: \n",mo_conf.ACC_En);
    }
    else if(GetMoConfData().ACC_En ==1)
    {
        mo_conf.ACC_En = (uint8_t)0;
        // ROS_INFO("Acc Enable Off\n");
        ROS_INFO("ACC_En: \n",mo_conf.ACC_En);
    }
}
// Service Function, if value is 0 -> value to be 1
void CAN::ConfEstopEnableStatus()
{
    if(mo_conf.ACC_aeb_seq ==0)
    {
        mo_conf.ACC_aeb_seq = (uint8_t)1;
        mo_conf.EPS_En = (uint8_t)0;
        mo_conf.ACC_En = (uint8_t)0;
    }
    else if(mo_conf.ACC_aeb_seq ==1)
    {
        mo_conf.ACC_aeb_seq =0;
    }
    // ROS_INFO("Estop Enable On\n");
}
// Service Function, if value is 0 -> value to be 1
void CAN::ConfSteerIgnoreStatus()
{
    if(GetMoConfData().EPS_IGNORE ==0)
    {
        mo_conf.EPS_IGNORE = (uint8_t)1;
        // ROS_INFO("Steer Ignore On\n");
    }
    else
    {
        mo_conf.EPS_IGNORE = (uint8_t)0;
        // ROS_INFO("Steer Ignore Off\n");
    }
}

//####################### Mode Status Topics ################################################
void CAN::VehicleStatus()
{
    //all_auto_status
    if(GetMoConfData().EPS_En ==1 && GetMoConfData().ACC_En==1)
    {
        AllEnableStatus.data = (uint8_t)1;
    }
    else
    {
        AllEnableStatus.data = (uint8_t)0;
    }
    // Estop Status
    if(GetMoConfData().ACC_aeb_seq ==1)
    {
        EstopStatus.data = (uint8_t)1;
    }
    else
    {
        EstopStatus.data = (uint8_t)0;
    }
    //EPS_Status
    if(GetMoConfData().EPS_En==1 && GetMoConfData().ACC_En==0)
    {
        EpsEnableStatus.data = (uint8_t)1;
    }
    else
    {
        EpsEnableStatus.data = (uint8_t)0;
    }
    //ACC_Enable Status
    if(GetMoConfData().ACC_En==1 && GetMoConfData().EPS_En==0)
    {
        AccEnableStatus.data =(uint8_t)1;
    }
    else
    {
        AccEnableStatus.data=(uint8_t)0;
    }
    //EPS_override Status
    if(GetMoConfData().EPS_IGNORE==1)
    {
        EpsIgnoreStatus.data=(uint8_t)1;
    }
    else
    {
        EpsIgnoreStatus.data=(uint8_t)0;
    }

}

// Parsing Data which will be published
void CAN::PubCmd()
{
    PubCmd156.ID = (uint32_t)0x156;
    PubCmd156.Length = (uint8_t)8;

    PubCmd156.Data.clear();
    PubCmd157.Data.clear();

    for(int i=0; i<8; i++)
    {
        PubCmd156.Data.push_back(0x00);
        PubCmd157.Data.push_back(0x00);
    }

    // std::cout << "ACC_aeb_seq : " << (int)GetMoConfData().ACC_aeb_seq << std::endl;

    PubCmd156.Data[0] = ((GetMoConfData().EPS_IGNORE & 0x0F)<<2) | ((GetMoConfData().EPS_En&0x0F));
    PubCmd156.Data[2] = ((GetMoConfData().ACC_aeb_seq) <<6) | ((GetMoConfData().ACC_StopRequest) <<4) | ((GetMoConfData().ACC_En));
    PubCmd156.Data[1] = ((uint8_t)(GetMoConfData().EPS_Slvel *1.0)) & 0xFF;
    PubCmd156.Data[3] = acc.ACC_Fd_VSpeed;//(uint8_t)(GetMoConfData().SetDispSpeed);
    PubCmd156.Data[5] = 0x00;
    PubCmd156.Data[7] = (uint8_t)(GetMoConfData().Mo_AlvCnt);

    PubCmd157.ID = (uint32_t)0x157;
    PubCmd157.Length = (uint8_t)8;
    // double Accel_Dec_Cmd = 1;
    unsigned short Acceleration_command = (mo_val.Accel_Dec_Cmd+10.23) * 100;
    PubCmd157.Data[0] = ((short)(mo_val.Steer_Cmd*10) & 0x00FF);
    PubCmd157.Data[1] = (((short)(mo_val.Steer_Cmd*10) & 0xFF00)>>8);
    PubCmd157.Data[3] = (unsigned char)(Acceleration_command & 0x00FF);
    PubCmd157.Data[4] = (unsigned char)((Acceleration_command & 0xFF00)>>8);
    // PubCmd157.Data[5] = ((uint8_t)(mo_val.Aeb_DecCmd*100) & 0xFF);

    mo_conf.Mo_AlvCnt++;
}


//####################### Publish Topics ###############################################################
void CAN::PublishTopics() //const ros::TimerEvent&
{
    //Change the ErrorBit Data as 0 or 1.
    EPSFdErrorCheck();
    AccFdErrorCheck();
    ModeCheck();
    VehicleStatus(); //Change Topic Variable Value.
    DigitText(); //Put the Data into the string
    PubCmd();//Publish Command into the Car through the CAN communication

    //if 0x710 & 0x711 both data are received Publish Topics.
    if(flag ==2)
    {
        //Publish ErrorBit
        eps_fd_errbit_pd_pub.publish(EpsErrorPd);
        eps_fd_errbit_eps_pub.publish(EpsErrorEps);
        eps_fd_errbit_vinfo_pub.publish(EpsErrorVinfo);
        eps_fd_errbit_sas_pub.publish(EpsErrorSas);
        acc_fd_errbit_tcu_pub.publish(AccErrorTcu);
        acc_fd_errbit_acc_pub.publish(AccErrorAcc);
        //Publish Status
        all_auto_status_pub.publish(AllEnableStatus);
        estop_status_pub.publish(EstopStatus);
        steering_status_pub.publish(EpsEnableStatus);
        acc_status_pub.publish(AccEnableStatus);
        steering_override_status_pub.publish(EpsIgnoreStatus);
        //Publish DigitText
        steer_cmd.publish(SteerCmdShow);
        accel_cmd.publish(AccelCmdShow);
        // status_string.publish(StatusStringShow);
        std_msgs::Float64 WheelSpeedAvg;
        WheelSpeedAvg.data = (wheel.WHL_SPD_FL + wheel.WHL_SPD_FR + wheel.WHL_SPD_RL + wheel.WHL_SPD_RR) / 4.;
        pub_speed.publish(WheelSpeedAvg);

        //Publish 156,157 Command
        Cmd156_pub.publish(PubCmd156);
        Cmd157_pub.publish(PubCmd157);



        flag =0;
    }

    vehicle_state_pub.publish(vehicle_state);

}

//####################### FeedBack ErrorBit Check & Set Topic Value for State of the Error & Set autonomous mode depends on the Errorbit conditions. ######################################################
void CAN::EPSFdErrorCheck()
{
    //Check EpsFdErrBitPd
    if(GetEpsData().EPS_Fd_ErrBit_pd ==0){
       EpsErrorPd.data = (uint8_t)0;
    }
    else{
       EpsErrorPd.data= (uint8_t)1;
       mo_conf.ACC_En = (uint8_t)0; //Disable the autonomous control mode
       mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control mode
       // mo_conf.ACC_aeb_seq = (uint8_t)1; //Request AEB
    }

    //Check EpsFdErrBitEPS
    if(GetEpsData().EPS_Fd_ErrBit_EPS==0){
        EpsErrorEps.data = (uint8_t)0;
    }
    else{
        EpsErrorEps.data = (uint8_t)1;
        mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control of the Steering Wheel
    }

    //Check EpsFdErrBitVinfo
    if(GetEpsData().EPS_Fd_ErrBit_Vinfo ==0){
        EpsErrorVinfo.data = (uint8_t)0;
    }
    else{
        EpsErrorVinfo.data = (uint8_t)1;
        mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control mode because of Error in the vehicle
        mo_conf.ACC_En = (uint8_t)0; //Disable the autonomous control mode because of Error in the vehicle
    }

    //Check EpsFdErrBitSas
    if(GetEpsData().EPS_Fd_ErrBit_SAS==0){
        EpsErrorSas.data = (uint8_t)0;
    }
    else{
        EpsErrorSas.data = (uint8_t)1;
        mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control mode becuase of Error in sensor of the Steering wheel
        mo_conf.ACC_En = (uint8_t)0; //Disable the autonomous control mode becuase of Error in sensor of the Steering wheel
    }
}

void CAN::AccFdErrorCheck()
{
    //Check AccFdErrBitTcu
    if(GetAccData().ACC_Fd_ErrBit_TCU==0){
        AccErrorTcu.data = (uint8_t)0;
    }
    else if(GetAccData().ACC_Fd_ErrBit_TCU==1)
    {
        AccErrorTcu.data = (uint8_t)1;
        mo_conf.ACC_En = (uint8_t)0; //Disable the autonomous control mode because of TCU module Error
        mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control mode because of TCU module Error
    }

    //Check AccFdErrBitAcc
    if(GetAccData().ACC_Fd_ErrBit_ACC==0){
        AccErrorAcc.data = (uint8_t)0;
    }
    else{
        AccErrorAcc.data = (uint8_t)1;
        mo_conf.ACC_En = (uint8_t)0; //Disable the autonomous control mode
        mo_conf.EPS_En = (uint8_t)0; //Disable the autonomous control mode
    }
}

void CAN::ModeCheck()
{
    //MoFdState
//    std::cout << "mo_state : " << (int)GetEpsData().Mo_Fd_State_EPS << std::endl;
    /*if(GetEpsData().Mo_Fd_State_EPS ==0)
    {
        mo_conf.EPS_En = (uint8_t)0;
        mo_conf.ACC_En = (uint8_t)0;
	std::cout << "test5" << std::endl;
    }*/

    //MoFdSeq
    if(GetEpsData().Mo_Fd_Seq_EPS>4)
    {
        mo_conf.EPS_En = (uint8_t)0;
        mo_conf.ACC_En = (uint8_t)0;
	//std::cout<<"test6" <<std::endl;
    }
}

// Digit Text Function
void CAN::DigitText()
{
    char Steer[sizeof(double)*8 +1];
    char Accel[sizeof(uint8_t)+1];
    char Status[20];

    //Status String
    if(GetEpsData().EPS_Fd_En == 1 && GetAccData().ACC_Fd_En ==1)
    {
        strcpy(Status,"All auto Mode");
    }
    else if( GetEpsData().EPS_Fd_En == 0 && GetAccData().ACC_Fd_En ==1)
    {
        strcpy(Status,"Acc auto Mode");
    }
    else if(GetEpsData().EPS_Fd_En == 1 && GetAccData().ACC_Fd_En ==0)
    {
        strcpy(Status,"Steering auto Mode");
    }
    else {
        strcpy(Status,"Manual Mode");
    }

    sprintf(Steer, "%f", eps.EPS_Fd_SteerAngle);
    sprintf(Accel, "%u", acc.ACC_Fd_VSpeed);

    string a= Steer;
    string b= Accel;

    SteerCmdShow.data = a;
    AccelCmdShow.data = b;
    StatusStringShow.data = Status;

}


double CAN::PIDController(double target, double current)
{

    //PID Calcluate Term
       float P_term = 0.0f;
       float I_term = 0.0f;
       float I_Error = 0.0f;
       float D_term = 0.0f;
       float error = 0.0f;
       float error_prev = 0.0f;
       float AntiWindupTerm = 0.0f;
       double PrevTarget = target;
       float ControlOutput = 0.0f;
       float ControlOutputBeforeSat = 0.0f;


       float Kp = 0.05;
       float Ki = 0.0;
       float Kd = 0.00;
       // static const float Ka = 1/Kp; // In most case, the K_a is equal to the 1 over Kp. It also can be in between 1/3Kp and 3/Kp //Anti-Windup Gain
       float dT = 0.1; //Make sure not 0
       float min = -3.0f; //Acceleration Min
       float max = 1.0f; // Acceleration Max



       error = target - current;
       cout << "Error Value: " << error << endl;
       P_term = Kp*error;
       I_Error += (error+AntiWindupTerm)*dT; //Cumulate the I_Error
       I_term = Ki*I_Error;
       D_term = Kd*(error-error_prev)/dT;
       // I_term = PIDSaturate(I_term, min, max);
       error_prev = error;
       if( PrevTarget != target)
       {
         I_term = 0.0;
       }
       //anti windup, output bound.
       //Reset output when I get the New target

       ControlOutput = P_term + I_term + D_term;
       ControlOutputBeforeSat = ControlOutput;
       ControlOutput = PIDSaturate(ControlOutput,min,max);
       cout << "Control OUTPUT: " << ControlOutput << endl;
       std::cout<< "Target: " << target << std::endl;
       std::cout<< "Current: " << current << std::endl;
       std::cout<< "P: " << P_term << std::endl;
       std::cout<< "D: " << D_term <<std::endl;
       // AntiWindupTerm = (ControlOutputBeforeSat - ControlOutput)*Ka;


   // else if(current < 40){

   //      float Kp = 0.15;
   //      float Ki = 0.0;
   //      float Kd = 0.03;
   //      // static const float Ka = 1/Kp; // In most case, the K_a is equal to the 1 over Kp. It also can be in between 1/3Kp and 3/Kp //Anti-Windup Gain
   //      float dT = 0.1; //Make sure not 0
   //      float min = -3.0f; //Acceleration Min
   //      float max = 1.0f; // Acceleration Max


   //     error = target - current;
   //     P_term = Kp*error;
   //     I_Error += (error+AntiWindupTerm)*dT; //Cumulate the I_Error
   //     I_term = Ki*I_Error;
   //     D_term = Kd*(error-error_prev)/dT;
   //     // I_term = PIDSaturate(I_term, min, max);
   //     error_prev = error;
   //     if( PrevTarget != target)
   //     {
   //       I_term = 0.0;
   //     }
   //     //anti windup, output bound.
   //     //Reset output when I get the New target

   //     ControlOutput = P_term + I_term + D_term;
   //     ControlOutputBeforeSat = ControlOutput;
   //     ControlOutput = PIDSaturate(ControlOutput,min,max);
   //     // AntiWindupTerm = (ControlOutputBeforeSat - ControlOutput)*Ka;
   // }


   return ControlOutput;
}

float CAN::PIDSaturate(double input, float min, float max)
{
   float saturated_value = input;
   if(input<min) // If speed is less than the minimum acceleration -> set output acceleration as the -3,0
   {
       saturated_value = min;
   }
   else if(input>max) // If acceleration is larger than maximum acceleration -> set output acceleration as the 1.0
   {
       saturated_value = max;
   }
    // printf("Target-based acceleration: %f\n", saturated_value);

   return saturated_value;
}

//############################## Subscriber & Service ###################################################
void CAN::CanCallback(const control_msgs::CanFrame::ConstPtr& Can)
{
    if(Can->ID == 0x710)
    {
        EPS_Parsing(Can->Data);
        flag = 1;
    }
    if(Can->ID == 0x711)
    {
        ACC_Parsing(Can->Data);
        flag++;
    }
    if(Can->ID == 0x712)
    {
        Wheel_Parsing(Can->Data);
        vehicle_state.header.seq += 1;
        vehicle_state.header.stamp = ros::Time::now();
        vehicle_state.header.frame_id = "/base_link";

        vehicle_state.v_ego = acc.ACC_Fd_VSpeed;
        vehicle_state.a_x = acc.LONG_ACCEL_ACC;
        vehicle_state.yaw_rate = ins.YAW_RATE;
        vehicle_state.steer_angle = eps.EPS_Fd_SteerAngle;

    }
    if(Can->ID == 0x713)
    {
        INS_Parsing(Can->Data);
    }

    PublishTopics();

}

void CAN::TargetVelocityCallback(const std_msgs::Float64::ConstPtr& TargetVelocity)
{
  double Target_Velocity = TargetVelocity->data;
  ROS_INFO("Tetsting\n");
  // double cur_speed = (wheel.WHL_SPD_FL + wheel.WHL_SPD_FR + wheel.WHL_SPD_RL + wheel.WHL_SPD_RR) / 4.;

  mo_val.Accel_Dec_Cmd = PIDController(Target_Velocity,acc.ACC_Fd_VSpeed);
}

void CAN::AccelerationCallback(const std_msgs::Float64::ConstPtr& Acceleration)
{
    float min = -3.0f; //Acceleration Min
    float max = 1.0f; // Acceleration Max
    double saturated_acc = PIDSaturate(Acceleration->data, min, max);
    printf("ACC-based acceleration: %f\n", saturated_acc);
    mo_val.Accel_Dec_Cmd = saturated_acc;
}

void CAN::SteerCmdCallback(const control_msgs::Vehicle_cmd::ConstPtr& Steering)
{
    // cout << Steering->target_steering <<endl;
    mo_val.Steer_Cmd = Steering->target_steering;
  //mo_val.Steer_Cmd = Steering->data;
}




bool CAN::all_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res)
{
    if(req.enable)
    {
        ConfAllEnableStatus();
    }

    res.response_str = "Done";
}

bool CAN::steer_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res)
{
    if(req.enable)
    {
        ConfSteerEnableStatus();
    }

    res.response_str = "Done";
}

bool CAN::acc_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res)
{
    if(req.enable)
    {
       ConfAccEnableStatus();
    }
    res.response_str = "Done";
}

bool CAN::estop_enable(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res)
{
    if(req.enable)
    {
        ConfEstopEnableStatus();
    }
    res.response_str = "Done";
}

bool CAN::steer_ignore(control_msgs::Vehicle_status::Request &req, control_msgs::Vehicle_status::Response &res)
{
    if(req.enable)
    {
        ConfSteerIgnoreStatus();
    }

    res.response_str = "Done";
}
