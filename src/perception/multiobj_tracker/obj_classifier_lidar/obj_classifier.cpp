#include "src/objclassifier.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_classifier");
    ObjClassiferLidar classifier;
    ros::spin();
    return 0;
}
