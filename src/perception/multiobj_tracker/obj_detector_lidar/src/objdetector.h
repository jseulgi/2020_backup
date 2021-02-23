#ifndef OBJ_DETECTOR_LIDAR_H
#define OBJ_DETECTOR_LIDAR_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include <obj_msgs/GridRoi.h>
#include <obj_msgs/Ego.h>
#include <obj_msgs/Obj.h>
#include <obj_msgs/ObjList.h>
#include <obj_msgs/ObjTrackBox.h>
#include <obj_msgs/ObjTrackBoxes.h>
#include <control_msgs/VehicleState.h>

#include <iostream>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>


typedef pcl::PointXYZI PointT;
const int num_region = 6;

class ObjDetectorLidar{
public:

    ObjDetectorLidar();
    // ~ObjDetectorLidar();
    void setParams();

    void egoStateCallback(const control_msgs::VehicleState &can_bus_msg);
    // void gpsCallback(const nav_msgs::Odometry &gps_msg);
    // void linkCallback(const  obj_msgs::Ego &link_msg);
    void gridroiCallback(const obj_msgs::GridRoi &gridroi_msg);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &sensor_pc);

    void cloudPrefilter(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc);
    void Crop3dROI(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc, Eigen::Vector4f Cubepoint_max, Eigen::Vector4f Cubepoint_min);

    void Voxelizer(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc, double leafsize);
    void RoadEliminator(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc);
    void ExtractInitialSeeds(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc);
    void Planefactorizer(pcl::PointCloud<PointT>::Ptr &input_pc);
    bool in_GridROI(PointT lidar_pnt, int true_value);
    void cloudClipper(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc1, pcl::PointCloud<PointT>::Ptr &output_pc2);

    void objectCluster(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc);
    void regionSeparator(pcl::PointCloud<PointT>::Ptr &input_pc, boost::array<std::vector<int>, num_region> &indicesArray_per_region);
    void cloudCluster(pcl::PointCloud<PointT>::Ptr &input_pc, std::vector<int> &input_indices, double tolerance, std::vector<pcl::PointIndices> &output_indicesSet);
    void objsPublisher(std::vector<pcl::PointCloud<PointT>> &clustered_objArray);
    double IoUCalculater(obj_msgs::ObjTrackBox target_bbox, obj_msgs::ObjTrackBox source_bbox);


private:
    ros::NodeHandle nh;

    ros::Publisher origin_pc_pub;  //transformed
    ros::Publisher prefiltered_pc_pub;     //nonground && in grid roi

    ros::Publisher ground_pc_pub;       //ground
    ros::Publisher ground_nonroi_pc_pub;
    ros::Publisher object_pc_pub;
    ros::Publisher obj_highint_pc_pub;
    ros::Publisher object_nonroi_pc_pub;    //nongrond && not in grid roi

    ros::Publisher obj_boundaries_pub;
    ros::Publisher intensity_marker_pub;

    /////////////////////////////
    ////params from yaml file////
    /////////////////////////////
    //sensor setting
    std::string sensor_model;
    double distance_interval;

    ////params for ROI
    double ROI_Front;
    double ROI_Back ;
    double ROI_Right;
    double ROI_Left ;
    double ROI_Top  ;
    double ROI_Bottom;
    Eigen::Vector4f ROI_minpoint;
    Eigen::Vector4f ROI_maxpoint;

    ////params for prefilte
    double voxel_grid_size;
    int voxel_minimum_size;
    double intensity_thread;

    ////params for Ground Plane Fitting
    int groundext_iter_num;
    int num_seg;
    int num_lpr;
    double seed_Hthd;
    double plane_Dthd;

    ////params for clustering
    int ec_minpnt_size;
    int ec_maxpnt_size;
    double init_tolerance;

    ////parameters to filter objects by their size
    bool use_size_filter;
    bool use_iou_combine;
    bool use_intensity_filter;
    double highI_rThd;
    double obj_minXscale;
    double obj_minYscale;
    double obj_minZscale;
    double obj_maxXscale;
    double obj_maxYscale;

    //ConvertBoxtoObj
    double egoLength;
    double egoWidth;

    /////////////////////////
    ////params for Global////
    /////////////////////////

    //Model parameter for ground plane fitting
    Eigen::MatrixXf plane_normal_;
    float plane_d;
    float ground_Dthd;

    //parameter setted by sensor model
    double vertical_res;        //depend on sensor model (OS1 -> 0.7)
    double tolerance_interval;  //depend on vertocal_res and distance_interval

    //parameter for tilting boxes
    double ego_speed = 0;
    double cur_speed = 0;
    double ego_yaw_rate = 0;
    double cur_yaw_rate = 0;

    //parameter for GridROi set
    obj_msgs::GridRoi temp_gridroi_msg;
    bool use_gridroi;
    bool is_gridroi = false;

    // ros::Time temp_gridroi_stamp;
    // double temp_diff_yaw_gridRoi;
    // double temp_grid_roi_resolution;
    // int temp_roi_value;
    // cv_bridge::CvImagePtr temp_gridroi_img;
    // cv::Mat temp_local_grid_roi;
    // sensor_msgs::PointCloud2 temp_link_points;

    ros::Time gridroi_stamp;
    double grid_roi_resolution;
    geometry_msgs::Point ego_link_point;
    double global_yaw_ref_map;
    int ego_link_type;
    int roi_value;
    cv::Mat local_grid_roi;
    sensor_msgs::PointCloud2 link_points;

};

#endif //OBJ_DETECTOR__LIDAR_H
