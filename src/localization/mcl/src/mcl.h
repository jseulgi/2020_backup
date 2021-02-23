#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <random>

#include <sensor_msgs/PointCloud2.h>

#include <jslocalization/node.h>
#include <jslocalization/pole.h>
#include <jslocalization/poles.h>
#include <jslocalization/planar.h>
#include <jslocalization/planars.h>
#include <jslocalization/mcl.h>
#include <jslocalization/ObjTrackBoxes.h>
#include <jslocalization/ObjTrackBox.h>
#include <jslocalization/PoleTrackBoxes.h>
#include <jslocalization/PoleTrackBox.h>
#include <jslocalization/PlanarTrackBoxes.h>
#include <jslocalization/PlanarTrackBox.h>
#include <jslocalization/FeatureBoxes.h>

#include <unavlib/convt.h>
#include <unavlib/others.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/MarkerArray.h>

#define SAVE_PARTICLES 0
struct particle{
      Eigen::Matrix4f pose;
      float score;
};

class mcl
{
    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_pub_mclPose;
        ros::Publisher m_pub_mclParticles;
        ros::Publisher m_pub_mclLiDAR;
        ros::Publisher m_pub_mclInfo;
        ros::Publisher result_pc_pub;
        ros::Publisher m_pub_markers;
        ros::Publisher m_pub_test;
        ros::Publisher m_pub_scan_planar;
        ros::Publisher m_pub_map_planar;
        ros::Publisher m_pub_pole;
        ros::Publisher m_pub_planar;
        ros::Publisher m_pub_map_pole;

        int m_param_particleMin;
        int m_param_particleMax;
        int m_param_pole_max;
        int m_param_planar_max;
        float m_static_velocity;
        float m_planar_cov;
        float m_theta_noise;
        
        Eigen::VectorXf m_param_cov;
        Eigen::Matrix4f m_bestpose;

        bool pole_map_received;
        bool planar_map_received;
        bool first_run;
        jslocalization::poles poles_map;
        jslocalization::planars planars_map;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pole_points_scan;
        pcl::PointCloud<pcl::PointXYZI>::Ptr selected_poles;        
        jslocalization::planars selected_planars;


        Eigen::Matrix4f m_predictPose;
        std::vector<particle> m_particles;
        // std::vector<double> weights; 

        std::vector<jslocalization::planars> vec_planars;

        std::vector<jslocalization::PlanarTrackBoxes> vec_tracked_planar;
        std::vector<jslocalization::PoleTrackBoxes> vec_tracked_pole;

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_planar_ptr;
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_planar_prt;

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_pole_ptr;


    public:
        mcl();   

        void SetParam();
        void timer_callback(const ros::TimerEvent& event);
        // void feature_callback(const jslocalization::feature::ConstPtr &feature);
        // void extract_closest_feature(std::vector<geometry_msgs::Point> pole_obs, pcl::PointCloud<pcl::PointXYZ> &out_cloud);
        void extract_closest_feature(jslocalization::ObjTrackBoxes poles, pcl::PointCloud<pcl::PointXYZ> &out_cloud);
        void extract_every_feature(jslocalization::ObjTrackBoxes poles, pcl::PointCloud<pcl::PointXYZ> &out_cloud);
        void extract_closest_pole(jslocalization::PoleTrackBoxes poles, pcl::PointCloud<pcl::PointXYZI> &out_cloud);
        // void extract_closest_planar(jslocalization::planars planars);
        // void extract_closest_planar(jslocalization::planars planars, jslocalization::planars &out_planars);
        void extract_closest_planar(jslocalization::PlanarTrackBoxes planars, jslocalization::planars &out_planars);



        void hdmap_pole_callback(const jslocalization::poles::ConstPtr &map);
        void hdmap_planar_callback(const jslocalization::planars::ConstPtr &map);

        void data_association_pole(pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud, pcl::PointXYZI observation, pcl::PointXYZI &closest_pole, int &closest_pole_id, float &min_dist);
        void data_association_planar(pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud, pcl::PointXYZI observation, pcl::PointXYZI &closest_planar, int &closest_planar_id, float &min_dist);
        void pole_callback(const jslocalization::ObjTrackBoxes::ConstPtr &msg);
        void pole_callback(const jslocalization::PoleTrackBoxes::ConstPtr &msg);
        void planar_callback(const jslocalization::planars::ConstPtr &msg);
        void planar_callback(const jslocalization::PlanarTrackBoxes::ConstPtr &msg);
        void feature_callback(const jslocalization::FeatureBoxes::ConstPtr &msg);
        // void initParticle(Eigen::Matrix4f initpose, float boundary, float boundary_angle);
        void initParticle(Eigen::Matrix4f initpose = Eigen::Matrix4f::Identity(), float boundary = 2, float boundary_angle = 5);

        particle spinOnce();
        std::vector<particle> prediction(std::vector<particle> particles, Eigen::MatrixXf trans);

        void weightning();
        float calcWeight_pole(float delta_dist, float covariance);
        float calcWeight_planar(float delta_ang, float covariance);


        void resampling();
        bool isOnmap(Eigen::MatrixXf pose);
        particle getBestParticle();
        int adapted_quantity();

        void pubparticle(particle particleo);
        void pubparticles(std::vector<particle> particles);
        void publidar(pcl::PointCloud<pcl::PointXYZ> lidar, Eigen::Matrix4f pose);
        void publidar(pcl::PointCloud<pcl::PointXYZI> lidar, Eigen::Matrix4f pose);

        float limitAngle(float input_angle);
        float limitAngle2(float input_angle);


};

#endif // MCL_H