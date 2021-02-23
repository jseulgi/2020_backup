#ifndef OBJ_CLASSIFIER_LIDAR_H
#define OBJ_CLASSIFIER_LIDAR_H

#include <ros/ros.h>
#include <ros/time.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>

#include <obj_msgs/Obj.h>
#include <obj_msgs/ObjList.h>
#include <obj_msgs/ObjTrackBox.h>
#include <obj_msgs/ObjTrackBoxes.h>

#include <iostream>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

class ObjClassiferLidar{
public:
    typedef enum OBJTYPE{
        NOINTEREST = 0,
        APPROACHING = 1,
        STATIC = 2,
        FRONTOBJ  = 3,
        OPPOSITE = 4
    } ObjType;

    typedef enum LINKTYPE{
        COUNTER_CLOCK_WISE = 50,
        INTERSECTION = 100,
        CLOCK_WISE = 150,
    } LINKTYPE;

    ObjClassiferLidar();

    void setParams();

    void trackboxCallback(obj_msgs::ObjTrackBoxes tracked_boxes);
    void objsPublisher(obj_msgs::ObjTrackBoxes &tracked_boxes);

    obj_msgs::Obj ConvertBoxtoObj(obj_msgs::ObjTrackBox &bbox);
    int DefineObjType(obj_msgs::Obj &object);
    bool is_static(obj_msgs::Obj &object);
    std::vector<geometry_msgs::Point> TilteBox(obj_msgs::Obj &object, double tilt_theta);
    double getNeighborPntsTheta(pcl::PointCloud<pcl::PointXYZI> &src_pc, float x_search, float y_search);
    void Kdtree(pcl::PointCloud<pcl::PointXYZI> &src_pc, float x_search, float y_search, std::vector<int> &searched_indices);


private:
    ros::NodeHandle nh;

    ros::Publisher obj_info_pub;
    ros::Publisher local_link_pc_pub;

    ros::Publisher obj_id_marker_pub;
    ros::Publisher obj_velocity_marker_pub;
    ros::Publisher obj_boundary_marker_pub;
    ros::Publisher obj_rawdetect_pub;

    //from yaml file
    double dist2egotail;
    double staticThd;
    bool use_velocity_fitting;
    double velocity_fitting_high_thd;
    double velocity_fitting_low_thd;

    //from msg
    pcl::PointCloud<pcl::PointXYZI> link_pnts;
    geometry_msgs::Point ego_link_point;
    int ego_link_type;
    double ego_link_diff_theta;
    double ego_current_speed;
    double ego_yaw_rate;
    double global_yaw_ref_map;
    double diff_yaw;
};


#endif //OBJ_CLASSIFIER_H
