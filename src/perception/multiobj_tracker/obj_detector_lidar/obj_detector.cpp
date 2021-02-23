#include "src/objdetector.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_detector");
    ObjDetectorLidar detector;
    ros::spin();
    return 0;
}
