# Parsed corrected IMU data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

uint32 gps_week_num
float64 gps_seconds

# All measurements in this message are instantaneous values;

float64 imu_status

# accelerations are in m/s
float64 accel_z
float64 accel_y
float64 accel_x

float64 gyro_z
float64 gyro_y
float64 gyro_x
