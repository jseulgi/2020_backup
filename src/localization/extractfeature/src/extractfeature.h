#ifndef EXTRACTFEATURE_H
#define EXTRACTFEATURE_H

#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <thread>

#include <sensor_msgs/PointCloud2.h>

#include <jslocalization/node.h>
#include <jslocalization/pole.h>
#include <jslocalization/ObjTrackBoxes.h>
#include <jslocalization/ObjTrackBox.h>
#include <jslocalization/planar.h>
#include <jslocalization/planars.h>
#include <jslocalization/PoleTrackBox.h>
#include <jslocalization/PoleTrackBoxes.h>
#include <jslocalization/PlanarTrackBox.h>
#include <jslocalization/PlanarTrackBoxes.h>
#include <jslocalization/FeatureBoxes.h>

#include <unavlib/convt.h>
#include <unavlib/others.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/common/centroid.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>


const int region_max_ = 6; // Change this value to match how far you want to detect.
typedef boost::array<std::vector<int>, region_max_> array8_t;
typedef std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> cluster_t;

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

class feature
{
    private:
        ros::NodeHandle m_nh;
        ros::Publisher xy_clipped_global_pub_;
        ros::Publisher non_ground_pub_;
        ros::Publisher marking_pub;

        ros::Publisher pole_local_pub_;
        ros::Publisher pl_raw_pub_;
        ros::Publisher pl_z_clipped_pub_;
        ros::Publisher pl_region_pub_;
        ros::Publisher pl_2d_pub_;
        ros::Publisher pl_2d_ransac_pub_;
        ros::Publisher plane_removed_pub_;

        ros::Publisher pole_global_pub_;
        ros::Publisher plane_global_pub_;
        ros::Publisher planar_tracking_pub_;
        ros::Publisher pole_tracking_pub_;
        ros::Publisher pole_obj_pub;
        ros::Publisher plane_obj_pub;
        ros::Publisher feature_obj_pub;

        ros::Publisher test_pub;
        ros::Publisher test1_pub_;
        ros::Publisher test2_pub_;
        ros::Publisher test3_pub_;
        ros::Publisher test4_pub_;
        ros::Publisher test5_pub_;
        ros::Publisher test6_pub_;

        Eigen::Matrix4f tf_lidar2body;

        std::string sensor_model_;

        // for ground extraction
        double sensor_height_;
        double removal_height_;
        int num_iter_;
        int num_lpr_;
        double th_seeds_;
        double th_dist_;
        double min_intensity;
        double max_intensity;
        float d_;
        MatrixXf normal_;
        float th_dist_d_;

        // for pole objects
        float po_x_width, po_y_width;
        float po_z_axis_max_, po_z_axis_min_;
        int po_cluster_size_min_, po_cluster_size_max_;
        float po_height;
        float po_height_max;
        float po_height_min;
        float po_voxel_size;
        float po_radius;
        float po_ec_tolerance;
        float po_trunk_height;

        // for plane objects
        float pl_z_axis_max_, pl_z_axis_min_; 
        int pl_cluster_size_min_, pl_cluster_size_max_;
        float pl_height;
        float pl_height_max;
        float pl_height_min;
        float pl_ratio;
        float pl_distance;
        int num_normal_kd;
        float normal_weight;

        int rg_num_ksearch;
        int rg_mincluster;
        int rg_numneighbor;
        float rg_smoothness;
        float rg_curvature;

        float ransac_threshold;
        int ransac_threads;
        float ransac_tolerance;
        int ransac_min;
        int ransac_max;

        float m_length_threshold;

        std::vector<Eigen::Vector4f> plane_cluster_centroid;
        std::vector<Eigen::Vector4f> pole_cluster_centroid;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pole;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_plane;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_marking;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pole_test;
    
        std::vector<jslocalization::PoleTrackBox> vec_pole;
        std::vector<jslocalization::PlanarTrackBox> vec_planar;

        int pole_cnt = 0;
        int planar_cnt = 0;


    public:
        feature();   
        void AllocateMemory();
        void ResetParam();
        void SetParam();
        void node_callback(const jslocalization::node::ConstPtr &nodegen);
        void road_plane_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud);
        void estimate_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud);
        void extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &out_cloud);
        void extract_plane_object(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Matrix4f pose_eigen,  Eigen::Matrix4f lidar_pose_eigen);
        void extract_pole_object(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Matrix4f pose_eigen, Eigen::Matrix4f lidar_pose_eigen);
        void remove_side_x(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range);
        void remove_side_y(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range);
        void clip_height(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double min_height, double max_height);
        void voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double leaf_x, double leaf_y, double leaf_z);
        std::vector<pcl::PointIndices> euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, float tolerance, int min_size, int max_size);
        void region_growing(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, std::vector<pcl::PointIndices> &out_cluster);

        array8_t create_array(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, double z_axis_min, double z_axis_max);
        void extract_inliers(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZI> &out_cloud);

        pcl::ModelCoefficients ransac_line(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, int threads, float threshold);
        void nearest_search(pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointXYZI src, float &min_dist);


};

#endif // EXTRACTFEATURE_H