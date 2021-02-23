# unavlib

### About
ROS 사용시 유용한 library

### ~~Install~~
~~'catkin_make inatll' instead of 'catkin_make'~~<br />

### Usage

* CmakeList.txt
``
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ...
  ...
  unavlib
)
``

* *.cpp
``
#include "unavlib/convt.h"
#include "unavlib/others.h"
using namespace unavlib;
``

### Library contents
* convert.h <br />
 - namespace cvt
``
Eigen::Matrix4f mat2eigen(cv::Mat mat);
cv::Mat eigen2mat(Eigen::Matrix4f mat);
void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);
Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat);
Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw);
std_msgs::String str2msgStr(std::string str);
std::string msgStr2str(std_msgs::String msgStr);
geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);
Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);
Eigen::Matrix4d eigenf2eigend(Eigen::Matrix4f pose);
Eigen::Matrix4f eigend2eigenf(Eigen::Matrix4d pose);
cv::Point eigen2cvpt(Eigen::MatrixXf pt);
Eigen::MatrixXf geoPoint2eigen(geometry_msgs::Point geoPoint);
geometry_msgs::Point eigen2geoPoint(Eigen::MatrixXf point);
void mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID = "map");
cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg);
pcl::PointCloud<pcl::PointXYZI> laser2cloud(sensor_msgs::LaserScan laser);
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZI> cloud);
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZRGB> cloud);
sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser);
std::vector<cv::Point2d> laser2cloud_cvpts(sensor_msgs::LaserScan laser);
nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt);
cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap);
void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in);
bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out);
bool LonLat2UTM(double lon, double lat, double *x, double *y, std::string zone);
void UTM2LonLat(double x, double y, double *lon, double *lat, std::string zone = "52S"); // 52S = daejeon
``
* others.h <br />
 - namespace probabilistic
``
double GaussianRand();
``
 - namespace time
``
unsigned long GetTickCount(int option = 0);   //0 : ms / 1:ns
void tic();
void toc();
``
 - namespace xtion
``
void update_caminfo(sensor_msgs::CameraInfo caminfo);
void update_range(double min = 0.35, double max = 0.5);
pcl::PointCloud<pcl::PointXYZRGB> get3D(cv::Mat img, cv::Mat depth);
pcl::PointCloud<pcl::PointXYZ> get3D(std::vector<float> pts, std::vector<float> depths);
``
 - namespace datahandle3d
``
cv::Vec3b heightcolor(double h);  // h : 0-1 -> color : blue - red
pcl::PointCloud<pcl::PointXYZ> voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size = 0.1);
pcl::PointCloud<pcl::PointXYZRGB> voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float size = 0.1);
pcl::PointCloud<pcl::PointXYZI> voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float size = 0.1);
``
