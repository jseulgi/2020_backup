#ifndef CONVT_H
#define CONVT_H

#include <vector>
#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

#include "utm.h"


namespace unavlib
{
  namespace cvt
  {
    Eigen::Matrix4f mat2eigen(cv::Mat mat);
    cv::Mat eigen2mat(Eigen::Matrix4f mat);
    void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
    cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);    
    Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat);
    Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw);
    Eigen::Matrix4f xyzrpy2eigen(Eigen::VectorXf xyzrpy);
    std_msgs::String str2msgStr(std::string str);
    std::string msgStr2str(std_msgs::String msgStr);
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);
    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);
    Eigen::Matrix4d eigenf2eigend(Eigen::Matrix4f pose);
    Eigen::Matrix4f eigend2eigenf(Eigen::Matrix4d pose);
    cv::Point eigen2cvpt(Eigen::MatrixXf pt);
    pcl::PointXYZ eigen2pcl(Eigen::MatrixXf pt);
    Eigen::MatrixXf pcl2eigen(pcl::PointXYZ pt);
    double interpolate(double x0, double x1, double ratio);
    geometry_msgs::Point interpolate(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1, double ratio);
    geometry_msgs::Quaternion interpolate(const geometry_msgs::Quaternion& quaternion0, const geometry_msgs::Quaternion& quaternion1, double ratio);
    geometry_msgs::Pose interpolate(const geometry_msgs::Pose& pose0, const geometry_msgs::Pose& pose1, double ratio);

    float twoPoints2dist(geometry_msgs::Point point1,geometry_msgs::Point point2);

    Eigen::MatrixXf geoPoint2eigen(geometry_msgs::Point geoPoint);
    geometry_msgs::Point eigen2geoPoint(Eigen::MatrixXf point);
    void mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID = "map");
    cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg);

    template<typename T>
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
    {
      sensor_msgs::PointCloud2 cloud_ROS;
      pcl::toROSMsg(cloud, cloud_ROS);
      cloud_ROS.header.frame_id = frame_id;
      return cloud_ROS;
    }

    template<typename T>
    pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
      pcl::PointCloud<T> cloudresult;
      pcl::fromROSMsg(cloudmsg,cloudresult);
      return cloudresult;
    }

    template<typename T>
    void cloudmsg2cloudptr(sensor_msgs::PointCloud2 cloudmsg,boost::shared_ptr< pcl::PointCloud< T > > cloudPtr)
    {
      pcl::fromROSMsg(cloudmsg,*cloudPtr);
    }


    template<typename T>
    pcl::PointCloud<T> laser2cloud(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pcl::PointCloud<T> cloud;
      pcl::fromROSMsg(cloud_ROS, cloud);
      return cloud;
    }


    sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser);
    std::vector<cv::Point2d> laser2cloud_cvpts(sensor_msgs::LaserScan laser);

    template<typename T>
    pcl::PointCloud<T> cloud2cloudcut(pcl::PointCloud<T> cloud_in,pcl::PointXYZ cloud_center,float min_dist,float max_dist)
    {
        pcl::PointCloud<T> cloud_return;
        for(int i=0;i<cloud_in.size();i++)
        {
            T cloud_pt = cloud_in.at(i);
            float dist = sqrt(pow(cloud_pt.x - cloud_center.x,2)+
                              pow(cloud_pt.y - cloud_center.y,2)+
                              pow(cloud_pt.z - cloud_center.z,2));
            if((dist>min_dist) && (dist<max_dist)) cloud_return.push_back(cloud_pt);
        }

        return cloud_return;
    }

    template<typename T>
    pcl::PointCloud<T> cloud2cloudcut(pcl::PointCloud<T> cloud_in, pcl::PointXYZ min_xyz, pcl::PointXYZ max_xyz)
    {
        pcl::PointCloud<T> cloud_return;
        for(int i=0;i<cloud_in.size();i++)
        {
            T cloud_pt = cloud_in.at(i);
            if(cloud_pt.x > min_xyz.x & cloud_pt.x < max_xyz.x &
               cloud_pt.y > min_xyz.y & cloud_pt.y < max_xyz.y &
               cloud_pt.z > min_xyz.z & cloud_pt.z < max_xyz.z)
            {
              // rejection point
            }
            else
              cloud_return.push_back(cloud_pt);
        }

        return cloud_return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloudRGBcut(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, float rMin,float rMax,float gMin,float gMax,float bMin,float bMax);

    Eigen::Matrix4f tf2eigen(tf::StampedTransform tf);
    tf::Transform geoPose2tf(geometry_msgs::Pose pose);

    nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt);
    cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap);
    void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in);
    bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out);

    bool LonLat2UTM(double lon, double lat, double *x, double *y, std::string zone);
    void UTM2LonLat(double x, double y, double *lon, double *lat, std::string zone = "52S"); // 52S = daejeon

    geometry_msgs::Pose tfstring2geoPose(std::string tfstring);
  }
}


#endif

