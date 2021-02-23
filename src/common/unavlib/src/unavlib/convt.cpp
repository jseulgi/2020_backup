#include "unavlib/convt.h"

namespace unavlib
{
  namespace cvt
  {
    float twoPoints2dist(geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        return sqrt(pow(point1.x-point2.x,2.0)+pow(point1.y-point2.y,2.0)+pow(point1.z-point2.z,2.0));
    }
    Eigen::Matrix4f mat2eigen(cv::Mat mat)
    {
        Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

        result(0,0) = mat.at<float>(0,0);
        result(0,1) = mat.at<float>(0,1);
        result(0,2) = mat.at<float>(0,2);
        result(0,3) = mat.at<float>(0,3);

        result(1,0) = mat.at<float>(1,0);
        result(1,1) = mat.at<float>(1,1);
        result(1,2) = mat.at<float>(1,2);
        result(1,3) = mat.at<float>(1,3);

        result(2,0) = mat.at<float>(2,0);
        result(2,1) = mat.at<float>(2,1);
        result(2,2) = mat.at<float>(2,2);
        result(2,3) = mat.at<float>(2,3);

        result(3,0) = mat.at<float>(3,0);
        result(3,1) = mat.at<float>(3,1);
        result(3,2) = mat.at<float>(3,2);
        result(3,3) = mat.at<float>(3,3);

        return result;
    }

    cv::Mat eigen2mat(Eigen::Matrix4f mat)
     {
         cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
         result.at<float>(0,0) = mat(0,0);
         result.at<float>(0,1) = mat(0,1);
         result.at<float>(0,2) = mat(0,2);
         result.at<float>(0,3) = mat(0,3);

         result.at<float>(1,0) = mat(1,0);
         result.at<float>(1,1) = mat(1,1);
         result.at<float>(1,2) = mat(1,2);
         result.at<float>(1,3) = mat(1,3);

         result.at<float>(2,0) = mat(2,0);
         result.at<float>(2,1) = mat(2,1);
         result.at<float>(2,2) = mat(2,2);
         result.at<float>(2,3) = mat(2,3);

         result.at<float>(3,0) = mat(3,0);
         result.at<float>(3,1) = mat(3,1);
         result.at<float>(3,2) = mat(3,2);
         result.at<float>(3,3) = mat(3,3);

         return result;
     }

    cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
    {
          cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
          rot_vec.at<float>(0) = roll;
          rot_vec.at<float>(1) = pitch;
          rot_vec.at<float>(2) = yaw;

          cv::Mat rot_mat;
          cv::Rodrigues(rot_vec,rot_mat);

          cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

          rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

          result.at<float>(0,3) = x;
          result.at<float>(1,3) = y;
          result.at<float>(2,3) = z;

          result.at<float>(3,3) = 1;

          return result;
    }

    void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
    {
        *x = mat.at<float>(0,3);
        *y = mat.at<float>(1,3);
        *z = mat.at<float>(2,3);

        cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

        cv::Mat rot_vec;
        cv::Rodrigues(rot_mat,rot_vec);
        *roll = rot_vec.at<float>(0);
        *pitch = rot_vec.at<float>(1);
        *yaw = rot_vec.at<float>(2);
    }




    Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat)
    {
        Eigen::VectorXf result(6);
        mat2xyzrpy(eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);
        return result;
    }


    Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
    {
        Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(x,y,z,roll,pitch,yaw));
        return result;
    }

    Eigen::Matrix4f xyzrpy2eigen(Eigen::VectorXf xyzrpy)
    {
      Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(xyzrpy(0),xyzrpy(1),xyzrpy(2),xyzrpy(3),xyzrpy(4),xyzrpy(5)));
      return result;
    }

    std_msgs::String str2msgStr(std::string str)
    {
      std_msgs::String msgStr;
      msgStr.data = str;
      return msgStr;
    }

    std::string msgStr2str(std_msgs::String msgStr)
    {
      return msgStr.data;
    }

    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose(0,0),
                (double)pose(0,1),
                (double)pose(0,2),
                (double)pose(1,0),
                (double)pose(1,1),
                (double)pose(1,2),
                (double)pose(2,0),
                (double)pose(2,1),
                (double)pose(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0,3);
        geoPose.position.y = pose(1,3);
        geoPose.position.z = pose(2,3);

        return geoPose;
    }

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose)
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 m(q);
        result(0,0) = m[0][0];
        result(0,1) = m[0][1];
        result(0,2) = m[0][2];
        result(1,0) = m[1][0];
        result(1,1) = m[1][1];
        result(1,2) = m[1][2];
        result(2,0) = m[2][0];
        result(2,1) = m[2][1];
        result(2,2) = m[2][2];
        result(3,3) = 1;

        result(0,3) = geoPose.position.x;
        result(1,3) = geoPose.position.y;
        result(2,3) = geoPose.position.z;

        return result;
    }

    Eigen::Matrix4d eigenf2eigend(Eigen::Matrix4f pose)
    {
      Eigen::Matrix4d result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    Eigen::Matrix4f eigend2eigenf(Eigen::Matrix4d pose)
    {
      Eigen::Matrix4f result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    cv::Point eigen2cvpt(Eigen::MatrixXf pt)
    {
      return cv::Point(pt(0),pt(1));
    }

    Eigen::MatrixXf geoPoint2eigen(geometry_msgs::Point geoPoint)
    {
      Eigen::MatrixXf result(4,1);
      result << geoPoint.x, geoPoint.y, geoPoint.z, 1;
      return result;
    }

    geometry_msgs::Point eigen2geoPoint(Eigen::MatrixXf point)
    {
      geometry_msgs::Point result;
      result.x = point(0);
      result.y = point(1);
      result.z = point(2);
      return result;
    }

    pcl::PointXYZ eigen2pcl(Eigen::MatrixXf pt)
    {
      pcl::PointXYZ result_xyz;
      result_xyz.x = pt(0,3);
      result_xyz.y = pt(1,3);
      result_xyz.z = pt(2,3);
      return result_xyz;
    }

    Eigen::MatrixXf pcl2eigen(pcl::PointXYZ pt)
    {
      Eigen::Matrix4f result(4,1);
      result << pt.x, pt.y, pt.z, 1;
      return result;
    }

    void mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID)
    {
        cv_bridge::CvImage bridge;
        mat.copyTo(bridge.image);
        bridge.header.frame_id = frameID;
        bridge.header.stamp = ros::Time::now();
        if(mat.type() == CV_8UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::MONO8;
        }
        else if(mat.type() == CV_8UC3)
        {
            bridge.encoding = sensor_msgs::image_encodings::BGR8;
        }
        else if(mat.type() == CV_32FC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }
        else if(mat.type() == CV_16UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        }
        else
        {
            std::cout <<"Error : mat type" << std::endl;

        }

        bridge.toImageMsg(sensorImg);
    }

    cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg)
    {
        static cv_bridge::CvImagePtr cv_ptr;
        cv::Mat mat;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
            mat = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        return mat;
    }



    pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
      pcl::PointCloud<pcl::PointXYZ> cloudresult;
      pcl::fromROSMsg(cloudmsg,cloudresult);
      return cloudresult;
    }

    sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      cloud_ROS.header.frame_id = "map";

      return cloud_ROS;
    }

    std::vector<cv::Point2d> laser2cloud_cvpts(sensor_msgs::LaserScan laser)
    {
      std::vector<cv::Point2d> result_pts;
      for(int i = 0; i < laser.ranges.size(); i++)
      {
        float dist = laser.ranges[i];
        float angle = laser.angle_min + laser.angle_increment * i;
        if(dist < laser.range_min || dist > laser.range_max)
          continue;
        cv::Point2d tmp_pt;
        tmp_pt.y = sin(angle) * dist;
        tmp_pt.x = cos(angle) * dist;
        result_pts.push_back(tmp_pt);

      }
      return result_pts;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloudRGBcut(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, float rMin,float rMax,float gMin,float gMax,float bMin,float bMax)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr PTCLIn(new pcl::PointCloud<pcl::PointXYZRGB>);
      *PTCLIn = cloud_in;
      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax+0.1)));
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin-0.1)));
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax+0.1)));
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin-0.1)));
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax+0.1)));
      condition->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin-0.1)));
      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
      condrem.setCondition (condition);
      condrem.setInputCloud (PTCLIn);
      // apply filter
      condrem.filter (*PTCLIn);
      return *PTCLIn;
    }


    Eigen::Matrix4f tf2eigen(tf::StampedTransform tf)
    {
      geometry_msgs::Pose tmp_pose;

      tmp_pose.position.x = tf.getOrigin().getX();
      tmp_pose.position.y = tf.getOrigin().getY();
      tmp_pose.position.z = tf.getOrigin().getZ();
      tmp_pose.orientation.x = tf.getRotation().getX();
      tmp_pose.orientation.y = tf.getRotation().getY();
      tmp_pose.orientation.z = tf.getRotation().getZ();
      tmp_pose.orientation.w = tf.getRotation().getW();
      return cvt::geoPose2eigen(tmp_pose);
    }

    tf::Transform geoPose2tf(geometry_msgs::Pose pose)
    {
      tf::Transform result;
      result.setOrigin(tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
      result.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
      return result;
    }

    nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt)
    {
      nav_msgs::OccupancyGrid m_gridmap;
      m_gridmap.info.resolution = resolution;
      geometry_msgs::Pose origin;
      origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
      origin.orientation.w = 1;
      m_gridmap.info.origin = origin;
      m_gridmap.info.width = cvimg.size().width;
      m_gridmap.info.height = cvimg.size().height;
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
      for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

      for(int y = 0; y < cvimg.size().height; y++)
      {
        for(int x = 0; x < cvimg.size().width; x++)
        {
          int tmpdata = cvimg.at<unsigned char>(y,x);
          int ttmpdata = -1; //Unknown
          if(tmpdata >= 150) //free
          {
            ttmpdata = (tmpdata - 250) / -2;
            if(ttmpdata < 0)
              ttmpdata = 0;
          }
          else if(tmpdata <= 98)
          {
            ttmpdata = (tmpdata - 200) / -2;
            if(ttmpdata > 100)
              ttmpdata = 100;
          }
          m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
        }
      }
      return m_gridmap;
    }

    cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap)
    {
      // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
      double resolution = occumap.info.resolution;
      cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                            -occumap.info.origin.position.y / resolution);
      cv::Size img_size;
      cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
      for(int pt = 0;pt < occumap.data.size();pt++)
      {
        int pt_y = pt / occumap.info.width;
        int pt_x = pt % occumap.info.width;
        int value = occumap.data.at(pt);
        unsigned char img_value;
        if(value == -1) img_value = 120;
        else if (value <= 50) img_value = 250 - 2 * value;
        else if (value >=51) img_value = 200 - 2 * value;
        cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
      }
      return cvimg;
    }

    void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in)
    {
      cv::Mat occumat = occumap2cvimg(gridmap_in);
      std::stringstream strm_png;
      strm_png << filepath << ".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";

      cv::imwrite(strm_png.str(),occumat);

      std::ofstream filesave(strm_info.str().c_str());
      if(filesave.is_open())
      {
        filesave << gridmap_in.info.resolution << "\n";
        filesave << gridmap_in.info.origin.position.x << "\n";
        filesave << gridmap_in.info.origin.position.y << "\n";
      }
      filesave.close();
    }

    bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out)
    {
      std::stringstream strm_png;
      strm_png << filepath <<".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";
      cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
      std::ifstream fileload(strm_info.str().c_str());
      float resolution,origin_x,origin_y;
      std::vector<std::string>   result;
      std::string line;
      if(!fileload.is_open())
      {
        std::cout << "Warning : Canot open occupancy map" << std::endl;
        return false;
      }
      while(std::getline(fileload,line)) result.push_back(line);
      if(result.size()!=3)
      {
        std::cout << "Warning : Canot open occupancy map (arguments)" << std::endl;
        return false;
      }
      resolution = std::atof(result.at(0).c_str());
      origin_x = std::atof(result.at(1).c_str());
      origin_y = std::atof(result.at(2).c_str());
      gridmap_out = cvimg2occumap(occumat, resolution,cv::Point(- origin_x / resolution,-origin_y / resolution));
      gridmap_out.header.frame_id = "map";

      return true;
    }



    bool LonLat2UTM(double lon, double lat, double *x, double *y, std::string zone)
    {
        char zone_char[100];
        UTM::LLtoUTM(lat, lon, *y, *x, zone_char);
        std::string str_zone = std::string(zone_char);
        if(str_zone.compare("52S") == 0)
        {
            return true;
        }
        return false;
    }

    void UTM2LonLat(double x, double y, double *lon, double *lat, std::string zone)
    {
        UTM::UTMtoLL(y, x, zone.c_str(), *lat, *lon);
    }

    geometry_msgs::Pose tfstring2geoPose(std::string tfstring)
    {
      geometry_msgs::Pose tfGeo;
      std::string::size_type sz;
      int szs = 0;
      tfGeo.position.x = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.position.y = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.position.z = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.orientation.x = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.orientation.y = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.orientation.z = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      tfGeo.orientation.w = std::stod (tfstring.substr(szs),&sz); szs+=sz;
      return tfGeo;
    }

    double interpolate(double x0, double x1, double ratio){
      return (1 - ratio) * x0 + ratio * x1;
    }
    geometry_msgs::Point interpolate(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1, double ratio){
        geometry_msgs::Point point_interpolated;
        point_interpolated.x = interpolate(p0.x, p1.x, ratio);
        point_interpolated.y = interpolate(p0.y, p1.y, ratio);
        point_interpolated.z = interpolate(p0.z, p1.z, ratio);
        return point_interpolated;
    }


    geometry_msgs::Quaternion interpolate(const geometry_msgs::Quaternion& quaternion0, const geometry_msgs::Quaternion& quaternion1, double ratio) {
      /**< SLERP denotes Spherical Linear interpolation  */

        tf::Quaternion q_interpolated;
        geometry_msgs::Quaternion q_out;

        tf::Quaternion q0;
        tf::quaternionMsgToTF(quaternion0, q0);
        tf::Quaternion q1;
        tf::quaternionMsgToTF(quaternion1, q1);

        // Normalize to avoid undefined behavior.
        q0.normalize();
        q1.normalize();

        double dot = q0.dot(q1);

        // If the dot product is negative, slerp won't take
        // the shorter path. Note that q1 and -q1 are equivalent when
        // the negation is applied to all four components. Fix by reversing one quaternion.
        if (dot < 0.0f) {
            q1 = q1 * (-1);
            dot = - dot;
        }
        const double DOT_THRESHOLD = 0.999;

        if (dot > DOT_THRESHOLD) {
            q_interpolated = q0 + (q1 - q0) * ratio;

        }else{
          // Since dot is in range [0, DOT_THRESHOLD], acos is safe
          q_interpolated = q0.slerp(q1, ratio);
        }

        q_interpolated.normalize();
        tf::quaternionTFToMsg(q_interpolated, q_out);
        return q_out;
    }

    geometry_msgs::Pose interpolate(const geometry_msgs::Pose& pose0, const geometry_msgs::Pose& pose1, double ratio){
      geometry_msgs::Pose pose_interpolated;
      pose_interpolated.orientation = interpolate(pose0.orientation, pose1.orientation, ratio);
      pose_interpolated.position = interpolate(pose0.position, pose1.position, ratio);
      return pose_interpolated;
    }
  }
}

