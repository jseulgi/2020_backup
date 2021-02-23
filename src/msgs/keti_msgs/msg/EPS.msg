# Messege that has a data related to steering angle from ECU
bool EPS_Fd_EN    # Manual/Auto mode transformation request: 0:disable, 1: enable
uint8 Mo_Fd_State
bool EPS_Fd_ErrBit_PD
bool EPS_Fd_ErrBit_EPS
bool EPS_Fd_ErrBit_VInfo
bool EPS_Fd_ErrBit_SAS
uint8 EPS_Mo_Fd_Seq
bool EPS_Fd_Ovrrd_Ignore
bool EPS_Fd_Override
float32 EPS_Fd_SteerAngle   # current steering angle
float32 EPS_Fd_Steer_Torq # steering torqe value
float32 EPS_Fd_Str_out_tq
#int8 EPS_Fd_Str_out_tq
uint8 Mo_Fd_AlvCnt





