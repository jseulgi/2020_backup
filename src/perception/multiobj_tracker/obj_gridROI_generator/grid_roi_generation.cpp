#include "src/gridROIgenerator.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "gridROI_generator");
    ObjGridRoiGenerator gridRoi;
    ros::spin();
    return 0;
}
