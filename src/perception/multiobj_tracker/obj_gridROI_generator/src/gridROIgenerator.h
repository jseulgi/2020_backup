#ifndef GRID_ROI_GENERATOR_H
#define GRID_ROI_GENERATOR_H

#include <ros/ros.h>
#include <ros/time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include </usr/include/gdal/ogrsf_frmts.h>
#include "path_msgs/Map.h"
#include <obj_msgs/GridRoi.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <string>

typedef pcl::PointXYZI PointT;

class ObjGridRoiGenerator{
public:
    typedef enum LINKTYPE{
    INTERSECTION = 0,
    CLOCK_WISE = 1,
    COUNTER_CLOCK_WISE = 2
    } LINKTYPE;

    ObjGridRoiGenerator();

    void setParams();

    void offsetCallback(const path_msgs::Map& offsett_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    void OpenLinkSHP();
    std::vector<geometry_msgs::Point> parsing_link_geometry(std::string geometry_str);

    void OpenLaneSHP();
    std::vector<geometry_msgs::Point> ParsingLaneGeometry(std::string geometry_str);
    std::string ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField);

    void genGridROI(double ego_x, double ego_y, double ego_yaw);
    void Kdtree(pcl::PointCloud<PointT> &search_pc, float x_search, float y_search, std::vector<int> &searched_indices);

private:
    ros::NodeHandle nh;

    ros::Publisher temp_local_link_map_pub;
    ros::Publisher local_link_map_pub;
    ros::Publisher local_lane_map_pub;
    ros::Publisher grid_roi_map_pub;
    ros::Publisher link_pc_pub;
    ros::Publisher lane_pc_pub;
    ros::Publisher grid_roi_pub;

    // update once in offsetCallback
    bool is_offsetVal = false;
    double mapX_offset=0;
    double mapY_offset=0;

    // update once in linkCallback
    bool is_linkCloud = false;
    pcl::PointCloud<PointT> globalLink_pc;
    pcl::PointCloud<PointT> globalLane_pc;

    // update once in odomCallback
    bool is_first_odom = true;
    double mapYaw_offset = 0;

    //update in odomCallback
    double diff_heading;

    //params from yaml
    std::string link_shp;
    std::string link_layer;
    std::string lane_shp;
    std::string lane_layer;

    int shp_link_type;

    double local_map_radius = 60.0;
    double map_resolution = 0.1;
    int ero_mask_pixel_size = 3;
    int dil_mask_pixel_size = 3;
    int lane_dil_mask_pixel_size = 3;
    int lane_ero_mask_pixel_size = 3;
};

#endif //GRID_ROI_GENERATOR_H
