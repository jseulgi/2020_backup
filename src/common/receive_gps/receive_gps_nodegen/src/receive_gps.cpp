#include "receive_gps.h"


GPS::GPS()
{

    Initialization();

}

GPS::~GPS(){
}

void GPS::Initialization(){
    map_initialized_ = false;
    subGpsIns = nh.subscribe("/inspva",1,&GPS::NovatelCallback, this);
    subMapInfo = nh.subscribe("/map_info", 1, &GPS::mapInfoCallback,this);
    pubGpsOdom = nh.advertise<nav_msgs::Odometry>("/gps_odom",1);
}

void GPS::mapInfoCallback(const path_msgs::Map msg)
{
  offset_x = msg.OffsetMapX;
  offset_y = msg.OffsetMapY;
  map_initialized_ = true;
}

void GPS::NovatelCallback(const novatel_gps_msgs::InspvaPtr& gpsHandler)
{
    if (map_initialized_)
    {
        double northing, easting;
        std::string zone;
        double NavheadingDeg = gpsHandler->azimuth;
        nav_msgs::Odometry odom;

        NavheadingDeg = -NavheadingDeg + 90;
        NavheadingRad = (NavheadingDeg * M_PI) / 180; // Yaw in Rad
        if(NavheadingRad > 2*M_PI) NavheadingRad -= 2*M_PI;
        else if(NavheadingRad < -2*M_PI) NavheadingRad += 2*M_PI;

        if(gpsHandler->header.stamp == ros::Time(0)){
            return;
        }

        LLtoUTM(gpsHandler->latitude, gpsHandler->longitude, northing, easting, zone);

        tf::Matrix3x3 heading_mat;
        heading_mat.setEulerYPR(NavheadingRad, 0, 0);

        tf::Quaternion q_tf;
        heading_mat.getRotation(q_tf);
        odom.pose.pose.position.x = easting - offset_x;
        odom.pose.pose.position.y = northing - offset_y;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = q_tf.getX();
        odom.pose.pose.orientation.y = q_tf.getY();
        odom.pose.pose.orientation.z = q_tf.getZ();
        odom.pose.pose.orientation.w = q_tf.getW();

        odom.header.frame_id = "map";
        odom.header.stamp = gpsHandler->header.stamp;

        // Send "base_link" transform to "map"
        tf::Transform transform;
        transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        pubGpsOdom.publish(odom);
        // ROS_INFO_STREAM("Odom is published");

    }
}
