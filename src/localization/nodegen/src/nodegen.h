#ifndef NODEGEN_H
#define NODEGEN_H

#include <ros/ros.h>
#include "conversions.h"
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/Inscov.h>
#include <novatel_gps_msgs/NovatelHeading2.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include "ogrsf_frmts.h"

#include <localization_msgs/node.h>
#include <localization_msgs/lidar.h>
#include <localization_msgs/pole.h>
#include <localization_msgs/poles.h>
#include <localization_msgs/planar.h>
#include <localization_msgs/planars.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#define CAR_WIDTH 1.5
#define CAR_LENGTH 3.0
#define EPSILON 0.00000000000001


struct GPS{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NodeGen
{
    private:
        ros::NodeHandle m_nh;
            
        // SetParam
        Eigen::Matrix4f tf_lidar2body;
        std::vector<sensor_msgs::PointCloud2> vector_lidar;  // LiDAR
        std::vector<nav_msgs::Odometry> vector_lidarodom;  // LiDAR
        std::vector<localization_msgs::lidar> vector_integrated_lidar;

        bool novatel_initialized = false;
        GPS hdmap_offset= {0,0,0,0,0,0};
        GPS init_guess = {0,0,0,0,0,0};
        float m_offset_x;
        float m_offset_y;

        nav_msgs::Odometry gps_odom;
        nav_msgs::Odometry lidar_odom;
        nav_msgs::Path lidarodom_path;

        std::vector<nav_msgs::Odometry> vector_gps_odom;
        std::vector<sensor_msgs::NavSatFix> vector_gps_fix;
        std::vector<novatel_gps_msgs::NovatelHeading2>vector_gps_heading;
        sensor_msgs::NavSatFix gps_covariance;

        double ratio; // Interpolate ratio

        // HD map
        std::string lane_shp;
        std::string lane_layer;
        std::string pole_shp;
        std::string pole_layer;
        std::string plane_shp;
        std::string plane_layer;
        std::string curb_shp;
        std::string curb_layer;
        ros::Publisher pole_marker_pub;
        ros::Publisher plane_marker_pub;
        ros::Publisher curb_marker_pub;
        visualization_msgs::Marker pole_marker;
        visualization_msgs::Marker plane_marker;
        visualization_msgs::Marker curb_marker;
        visualization_msgs::MarkerArray pole_array;
        visualization_msgs::MarkerArray plane_array;
        visualization_msgs::MarkerArray curb_array;
        pcl::PointCloud<pcl::PointXYZ> hdmap_pole_points;
        pcl::PointCloud<pcl::PointXYZI> hdmap_planar_points;
        pcl::PointCloud<pcl::PointXYZI> hdmap_curb_points;

        localization_msgs::poles poles;
        localization_msgs::planars planars;

        ros::Publisher node_pub;
        ros::Publisher gps_odom_pub;
        ros::Publisher gps_interpolated_pub;
        ros::Publisher hdmap_global_pub;
        ros::Publisher lidar_odom_pub;
        ros::Publisher lidar_global_pub;
        ros::Publisher lidar_points_pub; //for lego loam 
        ros::Publisher hdmap_pole_pub;
        ros::Publisher test_pole_pub;
        ros::Publisher test_planar_pub;
        ros::Publisher hdmap_planar_pub;

        std::vector<double> extRotV;
        Eigen::Matrix3d extRot;
        int odom_cnt=0;

    public:
        NodeGen();

        void SetParam();

        void callback_lidar(const sensor_msgs::PointCloud2::ConstPtr &lidar);
        void callback_inspva(const novatel_gps_msgs::Inspva::ConstPtr &inspva);
        void callback_gps(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void callback_gps_heading(const novatel_gps_msgs::NovatelHeading2::ConstPtr &novatel_heading);
        void callback_inscov(const novatel_gps_msgs::Inscov::ConstPtr &inscov);
        void callback_lidarodom(const nav_msgs::Odometry::ConstPtr &msg);

        void integrate_lidar();
        
        bool are_data_synced();
        bool are_inscov_synced();
        void gen_node();

        void get_ordered_pc(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst);
        void remove_unintended_parts(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst);
        void transform(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst, Eigen::Matrix4f& tf4x4);

        // HD Map LANE
        void open_lane_shp();
        void parsing_lane_geometry(std::vector<std::string> vector_lane_string, std::vector<geometry_msgs::Point> vector_lane_geo);
        void open_pole_shp();
        void open_plane_shp();
        std::string ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField);
        void pole_visualization(geometry_msgs::Point point, float radius);
        void pole_vis_initailization();
        void plane_visualization(std::vector<geometry_msgs::Point> vec_points);
};

#endif // NODEGEN_Hvoid NodeGen::transform(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst, Eigen::Matrix4f& tf4x4){

