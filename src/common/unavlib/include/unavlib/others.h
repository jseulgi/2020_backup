#ifndef OTHERS_H
#define OTHERS_H

#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include "sensor_msgs/CameraInfo.h"

namespace unavlib
{
  namespace probabilistic
  {
    double GaussianRand();
  }

  namespace time
  {
    unsigned long GetTickCount(int option = 0);   //0 : ms / 1:ns
    void tic();
    void toc();
    double Getdt(std_msgs::Header t1, std_msgs::Header t0);
  }

  namespace xtion
  {
    void update_caminfo(sensor_msgs::CameraInfo caminfo);
    void update_range(double min = 0.35, double max = 0.5);
    pcl::PointCloud<pcl::PointXYZRGB> get3D(cv::Mat img, cv::Mat depth, int resize = 1);
    pcl::PointCloud<pcl::PointXYZ> get3D(std::vector<float> pts, std::vector<float> depths);
    Eigen::MatrixXf get3D_eigen(std::vector<float> pts, std::vector<float> depths);
  }

  namespace datahandle3d
  {
    cv::Vec3b heightcolor(double h);  // h : 0-1 -> color : blue - red

    template <typename PointT>
    pcl::PointCloud<PointT> voxelize(boost::shared_ptr< pcl::PointCloud< PointT > > cloud, float size)
    {
      pcl::PointCloud<PointT> cloudvexel;

      pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (size, size, size);
      sor.filter (cloudvexel);

      return cloudvexel;
    }

    template <typename PointT>
    pcl::PointCloud<PointT> axiscut(boost::shared_ptr< pcl::PointCloud< PointT > > cloud, std::string axis, float min, float max)
    {
      pcl::PassThrough<PointT> pass;
      pcl::PointCloud <PointT> cloud_out;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName (axis);
      pass.setFilterLimits (min, max);
      pass.filter (cloud_out);
      return cloud_out;
    }

    template <typename PointT>
    pcl::PointCloud<PointT> voxelize_NoOverflow(boost::shared_ptr< pcl::PointCloud< PointT > > cloud, float leafsize)
    {
      boost::shared_ptr< pcl::PointCloud< PointT > > cloudin(new pcl::PointCloud< PointT >());
      *cloudin = axiscut(cloud, "z", -1, 30);

      pcl::PointXYZ min_p, max_p;
      min_p.x = std::numeric_limits<float>::max();
      min_p.y = std::numeric_limits<float>::max();
      min_p.z = std::numeric_limits<float>::max();
      max_p.x = std::numeric_limits<float>::min();
      max_p.y = std::numeric_limits<float>::min();
      max_p.z = std::numeric_limits<float>::min();
      for(int i = 0; i < cloudin->size(); i++)
      {
        PointT tmp_pt = cloudin->at(i);
        if(tmp_pt.x < min_p.x)
          min_p.x = tmp_pt.x;
        if(tmp_pt.x > max_p.x)
          max_p.x = tmp_pt.x;
        if(tmp_pt.y < min_p.y)
          min_p.y = tmp_pt.y;
        if(tmp_pt.y > max_p.y)
          max_p.y = tmp_pt.y;
        if(tmp_pt.z < min_p.z)
          min_p.z = tmp_pt.z;
        if(tmp_pt.z > max_p.z)
          max_p.z = tmp_pt.z;
      }

      pcl::PointCloud<PointT> cloudvoxel;

      double gridDist = (sqrt(std::numeric_limits<int32_t>::max() / ((32. / leafsize) + 1)) - 1) * leafsize; // max height = -1~30m
      for(float x = min_p.x; x <= max_p.x; x += gridDist)
      {
        boost::shared_ptr< pcl::PointCloud< PointT > > cloud_divX(new pcl::PointCloud<PointT>());
        *cloud_divX = axiscut(cloudin, "x", x, x + gridDist);

        for(float y = min_p.y; y <= max_p.y; y += gridDist)
        {
          boost::shared_ptr< pcl::PointCloud< PointT > > cloud_divXY(new pcl::PointCloud<PointT>());
          *cloud_divXY = axiscut(cloud_divX, "y", y, y + gridDist);
          pcl::VoxelGrid<PointT> sor;
          sor.setInputCloud (cloud_divXY);
          sor.setLeafSize (leafsize, leafsize, leafsize);
          pcl::PointCloud<PointT> cloudlocalvoxel;
          sor.filter (cloudlocalvoxel);
          cloudvoxel += cloudlocalvoxel;
        }
      }
      return cloudvoxel;
    }
  }


}


#endif

