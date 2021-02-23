#include "unavlib/others.h"

namespace unavlib
{
namespace probabilistic
{
double GaussianRand() {
  double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double r = u * u + v * v;
  if (r == 0 || r > 1) return GaussianRand();
  double c = sqrt(-2 * log(r) / r);
  return u * c;
}
}

namespace time
{
class __GET_TICK_COUNT
{
public:
  __GET_TICK_COUNT()
  {
    if (gettimeofday(&tv_, NULL) != 0)
      throw 0;
  }
  timeval tv_;
};
__GET_TICK_COUNT timeStart;

unsigned long GetTickCount(int option)   //0 : ms / 1:ns
{
  static time_t   secStart    = timeStart.tv_.tv_sec;
  static time_t   usecStart   = timeStart.tv_.tv_usec;
  timeval tv;
  gettimeofday(&tv, NULL);
  if(option == 0)
    return (tv.tv_sec - secStart) * 1000 + (tv.tv_usec - usecStart) / 1000;
  else
    return (tv.tv_sec - secStart) * 1000000 + (tv.tv_usec - usecStart);
}

    static unsigned long g_tictoctime = 0;
    void tic()
    {
      g_tictoctime = GetTickCount(1);
    }

    void toc()
    {
      unsigned long difftime = GetTickCount(1) - g_tictoctime;
      if(difftime < 1000)
        std::cout << "Tictoc : " << difftime << "ns" << std::endl;
      else if(difftime < 1000000)
      {
        double tmp = (double)difftime/1000;
        std::cout << "Tictoc : " << tmp << "ms" << std::endl;
      }
      else if(difftime < 1000000000)
      {
        double tmp = (double)difftime/1000000;
        std::cout << "Tictoc : " << tmp << "s" << std::endl;
      }
    }

    double Getdt(std_msgs::Header t1, std_msgs::Header t0)
    {
      int tmp_t_s = t1.stamp.sec - t0.stamp.sec;
      double dt_s = (double)tmp_t_s;
      int tmp_t_ns = t1.stamp.nsec - t0.stamp.nsec;
      double dt_ns = (double)tmp_t_ns / 1000000000;
      return dt_s + dt_ns;
    }
}

namespace xtion
{
static bool g_flag_caminfoupdate = false;
static Eigen::Matrix3f g_matK;
static double g_range_min = 0.35;
static double g_range_max = 0.5;

void update_caminfo(sensor_msgs::CameraInfo caminfo)
{
  g_flag_caminfoupdate = true;
  g_matK << caminfo.K[0], caminfo.K[1], caminfo.K[2],
      caminfo.K[3], caminfo.K[4], caminfo.K[5],
      caminfo.K[6], caminfo.K[7], caminfo.K[8];
}

void update_range(double min, double max)
{
  g_range_min = min;
  g_range_max = max;
}

pcl::PointCloud<pcl::PointXYZRGB> get3D(cv::Mat img, cv::Mat depth, int resize)
{
  pcl::PointCloud<pcl::PointXYZRGB> result;

  if(!g_flag_caminfoupdate)
  {
    std::cout << "Error : no K matrix" << std::endl;
    return result;
  }

  cv::Mat depthajust;
  if(img.size().width == depth.size().width)
    depthajust = depth.clone();
  else
    cv::resize(depth,depthajust,img.size());

  cv::Mat imgresize;
  cv::Mat depthresize;
  Eigen::Matrix3f matK;
  if(resize == 1)
  {
    imgresize = img.clone();
    depthresize = depthajust.clone();
    matK = g_matK;
  }
  else
  {
    cv::resize(img,imgresize,cv::Size(img.size().width/resize,img.size().height/resize));
    cv::resize(depthresize,depthajust,imgresize.size());
    matK << g_matK(0,0) /resize, g_matK(0,1) /resize, g_matK(0,2) /resize,
        g_matK(1,0) /resize, g_matK(1,1) /resize, g_matK(1,2) /resize,
        g_matK(2,0) /resize, g_matK(2,1) /resize, g_matK(2,2);
  }

  std::vector<float> pts;
  for(int y = 0; y < depthresize.size().height; y++)
  {
    for(int x = 0; x < depthresize.size().width; x++)
    {
      pts.push_back(x);
      pts.push_back(y);
    }
  }
  int pts_size = pts.size()/2;

  Eigen::MatrixXf pts1 = Eigen::Map<Eigen::Matrix<float,2,Eigen::Dynamic> > (pts.data(),2,pts_size);
  Eigen::MatrixXf pts1_ones = Eigen::Matrix<float,1,Eigen::Dynamic>(1,pts_size);
  pts1_ones.setOnes();
  Eigen::MatrixXf pts2 = Eigen::Matrix<float,3,Eigen::Dynamic>(3,pts_size);
  pts2 << pts1,
      pts1_ones;
  Eigen::MatrixXf pts2_Kinv = matK.inverse() * pts2;

  std::vector<float> depths;
  for(int y = 0; y < depthresize.size().height; y++)
  {
    for(int x = 0; x < depthresize.size().width; x++)
    {
      if(depthresize.type() == CV_16UC1)
        depths.push_back((float)depthresize.at<unsigned short>(y,x) / 1000);
      else
        depths.push_back(depthresize.at<float>(y,x));
    }
  }
  std::vector<cv::Vec3b> colors;
  for(int y = 0; y < imgresize.size().height; y++)
  {
    for(int x = 0; x < imgresize.size().width; x++)
    {
      colors.push_back(imgresize.at<cv::Vec3b>(y,x));
    }
  }

  Eigen::MatrixXf deps1 = Eigen::Map<Eigen::Matrix<float,1,Eigen::Dynamic> > (depths.data(),1,pts_size);
  Eigen::MatrixXf xyz(3,pts_size);
  xyz.row(0) = pts2_Kinv.row(0).cwiseProduct(deps1);
  xyz.row(1) = pts2_Kinv.row(1).cwiseProduct(deps1);
  xyz.row(2) = deps1;

  for(int i = 0; i < xyz.cols(); i++)
  {
    pcl::PointXYZRGB tmp_xyz;
    tmp_xyz.x = xyz(0,i);
    tmp_xyz.y = xyz(1,i);
    tmp_xyz.z = xyz(2,i);
    tmp_xyz.r = colors.at(i)[2];
    tmp_xyz.g = colors.at(i)[1];
    tmp_xyz.b = colors.at(i)[0];
    if(xyz(2,i) > g_range_min && xyz(2,i) < g_range_max)
      result.push_back(tmp_xyz);
  }
  return result;

}

pcl::PointCloud<pcl::PointXYZ> get3D(std::vector<float> pts, std::vector<float> depths)
{
  // [u,v,1]' = [K | 0] * [x,y,z,1]'

  pcl::PointCloud<pcl::PointXYZ> result;

  if(!g_flag_caminfoupdate)
  {
    std::cout << "Error : no K matrix" << std::endl;
    return result;
  }


  int pts_size = pts.size()/2;
  if(pts_size == 0)
  {
    //std::cout << "Error : pts size = 0" << std::endl;
    return result;
  }



  if(pts_size != depths.size())
  {
    std::cout << "Error : pts and depths size " << std::endl;
    return result;
  }


  //Eigen::Matrix<float,2,Eigen::Dynamic> a(pts.data());

  Eigen::MatrixXf pts1 = Eigen::Map<Eigen::Matrix<float,2,Eigen::Dynamic> > (pts.data(),2,pts_size);
  Eigen::MatrixXf pts1_ones = Eigen::Matrix<float,1,Eigen::Dynamic>(1,pts_size);
  pts1_ones.setOnes();
  Eigen::MatrixXf pts2 = Eigen::Matrix<float,3,Eigen::Dynamic>(3,pts_size);
  pts2 << pts1,
      pts1_ones;
  Eigen::MatrixXf pts2_Kinv = g_matK.inverse() * pts2;

  Eigen::MatrixXf deps1 = Eigen::Map<Eigen::Matrix<float,1,Eigen::Dynamic> > (depths.data(),1,pts_size);


  //Eigen::MatrixXf pts2_Kinv_div = 1/pts2_Kinv.row(2).array();

  Eigen::MatrixXf xyz(3,pts_size);
  xyz.row(0) = pts2_Kinv.row(0).cwiseProduct(deps1);
  xyz.row(1) = pts2_Kinv.row(1).cwiseProduct(deps1);
  xyz.row(2) = deps1;

  for(int i = 0; i < xyz.cols(); i++)
  {
    pcl::PointXYZ tmp_xyz;
    tmp_xyz.x = xyz(0,i);
    tmp_xyz.y = xyz(1,i);
    tmp_xyz.z = xyz(2,i);
    if(xyz(2,i) > g_range_min && xyz(2,i) < g_range_max)
      result.push_back(tmp_xyz);
  }
  return result;
}

Eigen::MatrixXf get3D_eigen(std::vector<float> pts, std::vector<float> depths)
{
  // [u,v,1]' = [K | 0] * [x,y,z,1]'

  Eigen::MatrixXf dummy_result(4,0);

  if(!g_flag_caminfoupdate)
  {
    std::cout << "Error : no K matrix" << std::endl;
    return dummy_result;
  }


  int pts_size = pts.size()/2;
  if(pts_size == 0)
  {
    std::cout << "Error : pts size = 0" << std::endl;
    return dummy_result;
  }



  if(pts_size != depths.size())
  {
    std::cout << "Error : pts and depths size " << std::endl;
    return dummy_result;
  }


  //Eigen::Matrix<float,2,Eigen::Dynamic> a(pts.data());

  Eigen::MatrixXf pts1 = Eigen::Map<Eigen::Matrix<float,2,Eigen::Dynamic> > (pts.data(),2,pts_size);
  Eigen::MatrixXf pts1_ones = Eigen::Matrix<float,1,Eigen::Dynamic>(1,pts_size);
  pts1_ones.setOnes();
  Eigen::MatrixXf pts2 = Eigen::Matrix<float,3,Eigen::Dynamic>(3,pts_size);
  pts2 << pts1,
      pts1_ones;
  Eigen::MatrixXf pts2_Kinv = g_matK.inverse() * pts2;

  Eigen::MatrixXf deps1 = Eigen::Map<Eigen::Matrix<float,1,Eigen::Dynamic> > (depths.data(),1,pts_size);


  //Eigen::MatrixXf pts2_Kinv_div = 1/pts2_Kinv.row(2).array();

  Eigen::MatrixXf xyz(4,pts_size);
  xyz.row(0) = pts2_Kinv.row(0).cwiseProduct(deps1);
  xyz.row(1) = pts2_Kinv.row(1).cwiseProduct(deps1);
  xyz.row(2) = deps1;
  xyz.row(3) = pts1_ones;
  return xyz;

  //      for(int i = 0; i < xyz.cols(); i++)
  //      {
  //        pcl::PointXYZ tmp_xyz;
  //        tmp_xyz.x = xyz(0,i);
  //        tmp_xyz.y = xyz(1,i);
  //        tmp_xyz.z = xyz(2,i);
  //        if(xyz(2,i) > g_range_min && xyz(2,i) < g_range_max)
  //          result.push_back(tmp_xyz);
  //      }
  //      return result;
}
}

namespace datahandle3d
{
cv::Vec3b heightcolor(double h)
{
  if(h > 1) h = 1;
  if(h < 0) h = 0;

  h = h * 0.667;


  double color_R;
  double color_G;
  double color_B;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 6:
  case 0:
    color_R = v; color_G = n; color_B = m;
    break;
  case 1:
    color_R = n; color_G = v; color_B = m;
    break;
  case 2:
    color_R = m; color_G = v; color_B = n;
    break;
  case 3:
    color_R = m; color_G = n; color_B = v;
    break;
  case 4:
    color_R = n; color_G = m; color_B = v;
    break;
  case 5:
    color_R = v; color_G = m; color_B = n;
    break;
  default:
    color_R = 1; color_G = 0.5; color_B = 0.5;
    break;
  }
  cv::Vec3b color;
  color[0] = color_R * 255;
  color[1] = color_G * 255;
  color[2] = color_B * 255;
  return color;
}


}


}
