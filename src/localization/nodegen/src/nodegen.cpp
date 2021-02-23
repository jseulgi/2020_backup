#include "nodegen.h"

using namespace unavlib;

NodeGen::NodeGen()
{
    SetParam();
    // static ros::Subscriber lidar_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/os1_cloud_node/points", 100, &NodeGen::callback_lidar, this);
    static ros::Subscriber lidar_sub2 = m_nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 100, &NodeGen::callback_lidar, this);    
    static ros::Subscriber inspva_sub = m_nh.subscribe<novatel_gps_msgs::Inspva>("/inspva", 100, &NodeGen::callback_inspva, this);
    static ros::Subscriber gps_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/fix",100,&NodeGen::callback_gps,this);
    static ros::Subscriber inscov_sub = m_nh.subscribe<novatel_gps_msgs::Inscov>("/inscov",100,&NodeGen::callback_inscov,this);

    static ros::Subscriber lidar_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("/integrated_to_init",100,&NodeGen::callback_lidarodom,this);

    node_pub = m_nh.advertise<localization_msgs::node>("/jslocalization/nodegen/node",100);

    gps_odom_pub = m_nh.advertise<nav_msgs::Odometry>("/jslocalization/nodegen/gps_raw", 10);
    gps_interpolated_pub =  m_nh.advertise<nav_msgs::Odometry>("/jslocalization/nodegen/gps_interpolated", 10);
    
    hdmap_global_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/nodegen/hdmap_global", 10);
    
    lidar_global_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/nodegen/lidar_global", 10);

    lidar_odom_pub = m_nh.advertise<nav_msgs::Odometry>("/jslocalization/nodegen/lidarodom",100);
    // lidar_odom_pub = m_nh.advertise<nav_msgs::Path>("/nodegen/debug/lidarodom",100);

    lidar_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/nodegen/points", 100); // to lego loam

    pole_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>("/jslocalization/nodegen/pole", 100);
    plane_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>("/jslocalization/nodegen/plane", 100);

    hdmap_pole_pub = m_nh.advertise<localization_msgs::poles>("/jslocalization/nodegen/pole_info", 100);
    hdmap_planar_pub = m_nh.advertise<localization_msgs::planars>("/jslocalization/nodegen/planar_info", 100);

    test_pole_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/nodegen/test_pole", 100); // for debug
    test_planar_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/nodegen/test_planar", 100); // for debug


}

void NodeGen::SetParam()
{
    std::vector<double> lidar2body;

    if(m_nh.getParam("/NodeGen/lidar2body", lidar2body))
    {
        if(lidar2body.size()==7){
            geometry_msgs::Pose geo_pose;
            geo_pose.position.x = lidar2body[0];
            geo_pose.position.y = lidar2body[1];
            geo_pose.position.z = lidar2body[2];
            geo_pose.orientation.x = lidar2body[3];
            geo_pose.orientation.y = lidar2body[4];
            geo_pose.orientation.z = lidar2body[5];
            geo_pose.orientation.w = lidar2body[6];
            tf_lidar2body = unavlib::cvt::geoPose2eigen(geo_pose);

        } else {
            ROS_ERROR("TF MUST BE X,Y,Z,Qx,Qy,Qw,Qz");
            exit(0);
        }
    }

    m_nh.param<std::string>("/PATH/lane_shp_path", lane_shp, "/home/jsg");
    m_nh.param<std::string>("/PATH/lane_shp_layer", lane_layer, "/home/jsg");
    m_nh.param<std::string>("/PATH/pole_shp_path", pole_shp, "/home/jsg");
    m_nh.param<std::string>("/PATH/pole_shp_layer", pole_layer, "/home/jsg");
    m_nh.param<std::string>("/PATH/plane_shp_path", plane_shp, "/home/jsg");
    m_nh.param<std::string>("/PATH/plane_shp_layer", plane_layer, "/home/jsg");
    m_nh.param<std::string>("/PATH/plane_shp_path", curb_shp, "/home/jsg");
    m_nh.param<std::string>("/PATH/plane_shp_layer", curb_layer, "/home/jsg");
    m_nh.param<std::vector<double>>("/legoloam/rotation", extRotV, std::vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);

    m_nh.param<float>("NodeGen/offset_x",m_offset_x,0.0); // offset btw hdmap and pointcloud map
    m_nh.param<float>("NodeGen/offset_y",m_offset_y,0.0);
}

void NodeGen::callback_lidar(const sensor_msgs::PointCloud2::ConstPtr &lidar)
{
    // static int cnt1 = 0;
    // std::string path1 = "/media/jsg/T7/temp_pcd/";
    // path1= path1 + "raw_" + std::to_string(cnt1) + ".pcd";
    // std::string path2 = "/media/jsg/T7/temp_pcd/";
    // path2 =  path2 + "tf_" + std::to_string(cnt1) + ".pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_raw (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar, *pcl_raw);
    pcl::transformPointCloud(*pcl_raw, *pcl_transformed, tf_lidar2body);
    sensor_msgs::PointCloud2 lidar_msg;
    pcl::toROSMsg(*pcl_transformed, lidar_msg );

    lidar_points_pub.publish(lidar_msg); // for legoloam
    vector_lidar.push_back(lidar_msg);    
    // vector_lidar.back().header.stamp = ros::Time::now();
    // std::cout << std::setprecision(20) << "point cloud size : " << vector_lidar.size() << " time: " <<lidar->header.stamp.toSec()<<std::endl;

    // if(cnt1==0){
        // pcl::io::savePCDFile<pcl::PointXYZI> (path1, *pcl_raw);
    //     pcl::io::savePCDFile<pcl::PointXYZI> (path2, *pcl_transformed);
    // }
    // cnt1=1;

}

void NodeGen::callback_gps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    vector_gps_fix.push_back(*msg);
}

void NodeGen::callback_inscov(const novatel_gps_msgs::Inscov::ConstPtr &inscov)
{
    // std::cout << "Callback inscov" << std::endl;
    gps_covariance.position_covariance =  inscov->position_covariance;
    // std::cout << gps_covariance << std::endl;
}

void NodeGen::callback_gps_heading(const novatel_gps_msgs::NovatelHeading2::ConstPtr &msg)
{
    vector_gps_heading.push_back(*msg);
}

void NodeGen::callback_inspva(const novatel_gps_msgs::Inspva::ConstPtr &inspva)
{
    std::string zone;
    GPS gps = {0, 0, 0, 0, 0, 0};
    GPS gps_global = {0, 0, 0, 0, 0, 0};

    ConvertGPS::LLtoUTM(inspva->latitude, inspva->longitude, gps.y, gps.x, zone);
    gps.z = inspva->height;

    gps.yaw= inspva->azimuth * M_PI / 180.0;
    gps.roll = inspva->roll * M_PI/ 180.0;
    gps.pitch = inspva->pitch * M_PI/ 180.0;

    if(novatel_initialized == false)
    {       
        open_lane_shp();
        open_pole_shp();
        open_plane_shp();

        init_guess.x = gps.x - hdmap_offset.x;
        init_guess.y = gps.y - hdmap_offset.y;
        init_guess.z = gps.z - hdmap_offset.z;
        init_guess.yaw = - gps.yaw + M_PI/2;

        novatel_initialized = true;

    }
    else
    {
        gps_global.x = gps.x - hdmap_offset.x;
        gps_global.y = gps.y - hdmap_offset.y;
        gps_global.z =  0; //gps.z - gps_init.z;

        gps_global.roll = 0; //gps.roll - gps_init.roll;
        gps_global.pitch = 0; //gps.pitch - gps_init.pitch;
        gps_global.yaw = gps.yaw; // - hdmap_offset.yaw;

        geometry_msgs::Pose gps_pose;
        Eigen::Matrix4f gps_eigen = cvt::xyzrpy2eigen(static_cast<float>(gps_global.x ), static_cast<float>(gps_global.y), static_cast<float>(gps_global.z),
                                                      static_cast<float>(0), static_cast<float>(0), static_cast<float>(-gps_global.yaw + M_PI/2 ));//+ M_PI));
        gps_pose = unavlib::cvt::eigen2geoPose(gps_eigen);
        // GPS odometry
        gps_odom.header.frame_id = "odom";
        gps_odom.header.stamp = inspva->header.stamp;
        gps_odom.pose.pose = gps_pose;
        gps_odom_pub.publish(gps_odom);

        vector_gps_odom.push_back(gps_odom);

        if(are_data_synced())
        {
            gen_node();
        }
    }
}

bool NodeGen::are_data_synced()
{
    while((vector_gps_odom.size() > 1) && (!vector_integrated_lidar.empty()))
    {
        double t_pc2 = vector_integrated_lidar.front().header.stamp.toSec();
        double t_odom = vector_gps_odom.front().header.stamp.toSec();
        double t_odom_next = vector_gps_odom.at(1).header.stamp.toSec();
        std::vector<double> times{t_pc2, t_odom};
        auto idxMM = std::minmax_element(times.begin(), times.end());


        if((t_pc2 > t_odom) && (t_pc2 < t_odom_next)){ // t_odom < t_pc2 < t_odom_next
            ratio = (t_pc2 - t_odom) / (t_odom_next - t_odom);

            return true;
        }
        else{
            if(idxMM.first - times.begin() == 0) vector_integrated_lidar.erase(vector_integrated_lidar.begin());
            else if (idxMM.first - times.begin() == 1) vector_gps_odom.erase(vector_gps_odom.begin());
        }

    }
    return false;
}

void NodeGen::gen_node()
{
    static int node_idx = 0;

    // std::cout << vector_gps_odom.size() << std::endl;
    // std::cout << vector_lidar[0].header.stamp.toSec() << std::endl;
    // std::cout << ratio << ", ";

    geometry_msgs::Pose interpose = unavlib::cvt::interpolate(vector_gps_odom[0].pose.pose, vector_gps_odom[1].pose.pose, ratio);

    nav_msgs::Odometry gps_interolated_msg;
    gps_interolated_msg.header.stamp = vector_lidar[0].header.stamp;
    gps_interolated_msg.header.frame_id = "odom";
    gps_interolated_msg.pose.pose = interpose;
    gps_interpolated_pub.publish(gps_interolated_msg);

    // Set node
    // 1. Interpolated pose
    static localization_msgs::node node;
    node.index = vector_integrated_lidar[0].index; 
    node.odom = vector_integrated_lidar[0].odom;
    node.gps_odom = interpose;
    node.odoms.push_back(vector_integrated_lidar[0].odom); 
    // node.odoms.push_back(interpose);

    // Set GPS Covariance
    // gps_covariance.latitude = interpose.position.x;
    // gps_covariance.longitude = interpose.position.y;
    // gps_covariance.altitude = interpose.position.z;
    node.gps = gps_covariance;

    // 2. Pointcloud w.r.t body frame
    pcl::PointCloud<pcl::PointXYZI> pc2 = cvt::cloudmsg2cloud<pcl::PointXYZI>(vector_integrated_lidar[0].lidar);
    // std::cout<<pc2.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZI> pc2_ordered;
    get_ordered_pc(pc2, pc2_ordered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc2_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(pc2_ordered, *pc2_transformed, tf_lidar2body);  

    node.lidar = cvt::cloud2msg(*pc2_transformed);
    node.lidar.header.stamp = vector_integrated_lidar[0].header.stamp;
    node.lidar.header.frame_id = "odom";
    node_pub.publish(node);

    std::cout << "\033[1;33m[NodeGen]\033[0m : Complet to publish "<<node_idx<<"th node."<<std::endl;
    ++node_idx;
    vector_gps_odom.erase(vector_gps_odom.begin(), vector_gps_odom.begin() + 1);
    vector_integrated_lidar.erase(vector_integrated_lidar.begin());

    // for visualization
    // pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_global (new pcl::PointCloud<pcl::PointXYZI>);
    // Eigen::Matrix4f odom_eigen = cvt::geoPose2eigen(node.odom);
    // Eigen::Matrix4f gps_eigen = cvt::geoPose2eigen(interpose);
    // pcl::transformPointCloud(pc2_transformed, *lidar_global, odom_eigen);
    // pcl::transformPointCloud(*pc2_transformed, *lidar_global, gps_eigen);
    // lidar_global_pub.publish(cvt::cloud2msg(*lidar_global, "odom"));
}

void NodeGen::get_ordered_pc(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
    pcl::PointCloud<pcl::PointXYZI> tmp;
    transform(src, tmp, tf_lidar2body);
    remove_unintended_parts(tmp, dst);
}

void NodeGen::remove_unintended_parts(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
    for (auto const &pt : src.points){
        if ( (abs(pt.x) < CAR_LENGTH/1.3) && (abs(pt.y) < CAR_WIDTH/3) ){
            continue;
        }else{
            dst.push_back(pt);
        }
    }
}

void NodeGen::transform(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst, Eigen::Matrix4f& tf4x4){
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(src, *ptr_transformed, tf4x4);
    dst = *ptr_transformed;
}

void NodeGen::open_lane_shp()
{
    OGRRegisterAll();
    std::vector<std::string> vector_lane_string;
    std::vector<geometry_msgs::Point> vector_lane_geo;

    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx(lane_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL));

    if(poDS == NULL)
    {
        std::cout << "\033[1;31mHD Map Open Failed\033[0m" << std::endl;
        std::exit(1);
    }

    std::cout << "\033[1;33m[NodeGen]\033[0m : HD Map Open Successed" << std::endl;

    OGRLayer *poLayer = poDS->GetLayerByName(lane_layer.c_str());
    // OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;

    while((poFeature = poLayer->GetNextFeature()) != NULL){
        char *pszWKT = NULL;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
//        std::cout << wkbFlatten(poGeometry->getGeometryType()) << std::endl;
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString){
            poGeometry->exportToWkt(&pszWKT);
//            std::cout << pszWKT << std::endl;
            vector_lane_string.push_back(pszWKT);
            CPLFree(pszWKT);
        } else {
            std::cout << "LANE : No Point Geometry" << std::endl;
        }

        OGRFeature::DestroyFeature(poFeature);
    }

    GDALClose(poDS);
    parsing_lane_geometry(vector_lane_string, vector_lane_geo);

}

void NodeGen::parsing_lane_geometry(std::vector<std::string> vector_lane_string, std::vector<geometry_msgs::Point> vector_lane_geo)
{
    pcl::PointXYZ hdmap_lane_points;
    geometry_msgs::Point lane_point;
    std::string string_geometry;
    std::string string;

    unsigned long comma, space;
    pcl::PointCloud<pcl::PointXYZ> pc_hdmap_lane;

    // static double azimuth_offset = 90.0 * M_PI / 180.0;

    for(size_t i=0; i<vector_lane_string.size(); i++)
    {
        string_geometry = vector_lane_string[i];
        comma = string_geometry.find("(");
        string_geometry = string_geometry.substr(comma+1);

        while(string_geometry.find(",") != std::string::npos)
        {
            comma = string_geometry.find(",");
            string = string_geometry.substr(0,comma);
            space = string.find(" ");
            lane_point.x = atof(string.substr(0,space).c_str());
            string = string.substr(space+1);
            space = string.find(" ");
            lane_point.y = atof(string.substr(0,space).c_str());
            // string = string.substr(space+1);
            // space = string.find(" ");
            // lane_point.z = atof(string.substr(0,space).c_str());
            lane_point.z = 0;

            vector_lane_geo.push_back(lane_point);
            string_geometry = string_geometry.substr(comma+1);
        }

        space = string_geometry.find(" ");
        lane_point.x = atof(string_geometry.substr(0,space).c_str()) ;
        string_geometry = string_geometry.substr(space+1);
        space = string_geometry.find(" ");
        lane_point.y = atof(string_geometry.substr(0,space).c_str());
        // string_geometry = string_geometry.substr(space+1);
        // space = string_geometry.find(" ");
        // lane_point.z = atof(string_geometry.substr(0,space).c_str());
        lane_point.z = 0;

        vector_lane_geo.push_back(lane_point);

        for(size_t j = 0; j< vector_lane_geo.size(); j++)
        {

            if(i==0 && j==0)
            {
                hdmap_offset.x = vector_lane_geo[0].x;
                hdmap_offset.y = vector_lane_geo[0].y;
                hdmap_offset.z = vector_lane_geo[0].z;
                std::cout << "\033[1;33m[NodeGen]\033[0m : HDmap offset x : " << hdmap_offset.x << " y: "<<hdmap_offset.y<<" z: "<<hdmap_offset.z<<std::endl;
            }

            vector_lane_geo[j].x = vector_lane_geo[j].x - hdmap_offset.x;//hdmap_init.x; 
            vector_lane_geo[j].y = vector_lane_geo[j].y - hdmap_offset.y;//hdmap_init.y; 
            vector_lane_geo[j].z = vector_lane_geo[j].z - hdmap_offset.z;//hdmap_init.z; 
            // double x_curr = vector_lane_geo[j].x;
            // double y_curr = vector_lane_geo[j].y;
            // double z_curr = vector_lane_geo[j].z;
            // double heading_curr =  hdmap_offset.yaw; 

            // vector_lane_geo[j].x = x_curr * cos(heading_curr - azimuth_offset) - y_curr * sin(heading_curr - azimuth_offset);
            // vector_lane_geo[j].y = x_curr * sin(heading_curr - azimuth_offset) + y_curr * cos(heading_curr - azimuth_offset);

            hdmap_lane_points.x = vector_lane_geo[j].x;
            hdmap_lane_points.y = vector_lane_geo[j].y;
            hdmap_lane_points.z = 0;//vector_lane_geo[j].z;

            pc_hdmap_lane.points.push_back(hdmap_lane_points);
        }
        vector_lane_geo.clear();
    }
    hdmap_global_pub.publish(unavlib::cvt::cloud2msg(pc_hdmap_lane, "odom"));
}

void NodeGen::open_pole_shp()
{
    OGRRegisterAll();
    std::vector<std::string> vector_pole_string;
    std::vector<geometry_msgs::Point> vector_pole_geo;

    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx(pole_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL));

    if(poDS == NULL){
        std::cout << "\033[1;31mPole map Open Failed\033[0m" << std::endl;
        std::exit(1);
    }

    std::cout << "\033[1;33m[NodeGen]\033[0m : Pole Map Open Successed" << std::endl;

    OGRLayer *poLayer = poDS->GetLayerByName(pole_layer.c_str());
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;

    std::vector<std::string> vec_idx;
    std::vector<std::string> vec_radius;
    std::vector<std::string> vec_geo_pole;

    while((poFeature = poLayer->GetNextFeature()) != NULL)
    {
        for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
        {
            OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
            std::string str = "";
            str = ParsingToString( poFieldDefn, poFeature, iField);

            if(iField==0)
                vec_idx.push_back(str);
            else if(iField==1)
                vec_radius.push_back(str);
        }

        char *pszWKT = NULL;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
//        std::cout << wkbFlatten(poGeometry->getGeometryType()) << std::endl;
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbPoint){
            poGeometry->exportToWkt(&pszWKT);
//            std::cout << pszWKT << std::endl;
            vec_geo_pole.push_back(pszWKT);
            CPLFree(pszWKT);
        } else {
            std::cout << "Pole : No Point Geometry" << std::endl;
        }
        OGRFeature::DestroyFeature(poFeature);
    }

    GDALClose(poDS);

    // Parsing //   POINT (353149.556060791 4025885.27337646)  0.638969659805298
    geometry_msgs::Point shp_point;
    std::vector<geometry_msgs::Point> vec_pole_points;
    std::string geometry_str;
    std::string str;
    unsigned long comma;
    unsigned long space;

    pcl::PointXYZ points;


    for(int i=0; i<vec_geo_pole.size(); i++)
    {
        comma = vec_geo_pole[i].find("(");
        geometry_str = vec_geo_pole[i].substr(comma+1);
        comma = geometry_str.find(" ");
        str = geometry_str.substr(0,comma);
        shp_point.x = atof(str.c_str()) - hdmap_offset.x - m_offset_x;
        geometry_str = geometry_str.substr(comma+1);
        comma = geometry_str.find(")");
        str = geometry_str.substr(0,comma);
        shp_point.y = atof(str.c_str()) - hdmap_offset.y - m_offset_y;
        shp_point.z = 0.0; // z에 0이 아닌 다른 값을 넣으면 mcl에서 터짐 
        vec_pole_points.push_back(shp_point);
        // pole_marker.points.push_back(shp_point);

        points.x = shp_point.x;
        points.y = shp_point.y;
        points.z = shp_point.z;
        hdmap_pole_points.points.push_back(points);
    }
    

    for(int i=0; i<vec_pole_points.size(); i++)
    {
        pole_visualization(vec_pole_points[i], atof(vec_radius[i].c_str()));

        localization_msgs::pole pole_info;
        pole_info.header.frame_id = "odom";
        pole_info.header.stamp = ros::Time::now();
        pole_info.index = i;
        pole_info.centroid = vec_pole_points[i];
        pole_info.radius = atof(vec_radius[i].c_str());
        pole_info.covariance = pow(pole_info.radius,2)/5.9915;
        poles.pole.push_back(pole_info);
    }

    pole_marker_pub.publish(pole_array);

    poles.pc_pole = cvt::cloud2msg(hdmap_pole_points);
    hdmap_pole_pub.publish(poles);

    test_pole_pub.publish(cvt::cloud2msg(hdmap_pole_points, "odom"));
}

std::string NodeGen::ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField)
{

    std::string str = "";
    switch( poFieldDefn->GetType() )
    {
        case OFTInteger:
            // printf( "%d,", poFeature->GetFieldAsInteger( iField ) );
            str.append(std::to_string( poFeature->GetFieldAsInteger(iField) ) );
            break;
        case OFTInteger64:
            // printf( CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64( iField ) );
            str.append(std::to_string( poFeature->GetFieldAsInteger64(iField) ) );
            break;
        case OFTReal:
            // printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
            str.append(std::to_string( poFeature->GetFieldAsDouble(iField) ) );
            break;
        case OFTString:
            // printf( "%s,", poFeature->GetFieldAsString(iField) );
            str.append( poFeature->GetFieldAsString(iField ) );
            break;
        default:
            // printf( "%s,", poFeature->GetFieldAsString(iField) );
            str.append( poFeature->GetFieldAsString(iField) );
            break;
    }
    return str;
}

void NodeGen::pole_visualization(geometry_msgs::Point point, float radius)
{
    static int cnt = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    // marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = cnt++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 3.0 * radius;
    marker.scale.y = 3.0 * radius;
    marker.scale.z = 0.001;
    marker.color.a = 0.7; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    pole_array.markers.push_back(marker);
}

void NodeGen::pole_vis_initailization(){
    pole_marker.header.frame_id = "odom";
    pole_marker.header.stamp = ros::Time::now();
    pole_marker.action = visualization_msgs::Marker::ADD;
    pole_marker.pose.orientation.w = 1.0;
    pole_marker.id = 1000;
    pole_marker.type = visualization_msgs::Marker::POINTS;
    pole_marker.scale.x = 1.5;
    pole_marker.scale.y = 1.0;
    pole_marker.color.r = 1.0;
    pole_marker.color.g = 1.0;
    pole_marker.color.b = 0.0;
    pole_marker.color.a = 1.0;        
}

void NodeGen::open_plane_shp()
{
    OGRRegisterAll();
    std::vector<std::string> vector_plane_string;
    std::vector<geometry_msgs::Point> vector_plane_geo;

    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx(plane_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL));

    if(poDS == NULL){
        std::cout << "\033[1;31mPlnae map Open Failed\033[0m" << std::endl;
        std::exit(1);
    }
    std::cout << "\033[1;33m[NodeGen]\033[0m : Plane Map Open Successed" << std::endl;

    OGRLayer *poLayer = poDS->GetLayerByName(plane_layer.c_str());
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;

    std::vector<std::string> vec_idx;
    std::vector<std::string> vec_start_x;
    std::vector<std::string> vec_start_y;
    std::vector<std::string> vec_end_x;
    std::vector<std::string> vec_end_y;
    std::vector<std::string> vec_centroid_x;
    std::vector<std::string> vec_centroid_y;
    std::vector<std::string> vec_length;
    std::vector<std::string> vec_theta;
    std::vector<std::string> vec_geo_plane;

    while((poFeature = poLayer->GetNextFeature()) != NULL)
    {
        for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
        {
            OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
            std::string str = "";
            str = ParsingToString( poFieldDefn, poFeature, iField);

            if(iField==0)
                vec_idx.push_back(str);
            else if(iField==1)
                vec_start_x.push_back(str);
            else if(iField==2)
                vec_start_y.push_back(str);
            else if(iField==3)
                vec_end_x.push_back(str);
            else if(iField==4)
                vec_end_y.push_back(str);
            else if(iField==5)
                vec_centroid_x.push_back(str);
            else if(iField==6)
                vec_centroid_y.push_back(str);
            else if(iField==7)
                vec_length.push_back(str);
            else if(iField==8)
                vec_theta.push_back(str);
        }

        char *pszWKT = NULL;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
//        std::cout << wkbFlatten(poGeometry->getGeometryType()) << std::endl;
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString){
            poGeometry->exportToWkt(&pszWKT);
//            std::cout << pszWKT << std::endl;
            vec_geo_plane.push_back(pszWKT);
            CPLFree(pszWKT);
        } else {
            std::cout << "Plane : No Point Geometry" << std::endl;
        }
        OGRFeature::DestroyFeature(poFeature);
    }

    GDALClose(poDS);

    geometry_msgs::Point shp_point;
    std::string geometry_str;
    std::string str;
    unsigned long comma;
    unsigned long space;

    for(int i=0; i<vec_geo_plane.size(); i++)
    {
        std::vector<geometry_msgs::Point> vec_plane_points;
        comma = vec_geo_plane[i].find("(");
        geometry_str = vec_geo_plane[i].substr(comma+1);
        comma = geometry_str.find(" ");
        str = geometry_str.substr(0,comma);
        shp_point.x = atof(str.c_str()) - hdmap_offset.x - m_offset_x;
        geometry_str = geometry_str.substr(comma+1);
        comma = geometry_str.find(",");
        str = geometry_str.substr(0,comma);
        shp_point.y = atof(str.c_str()) - hdmap_offset.y - m_offset_y;
        vec_plane_points.push_back(shp_point);

        geometry_str = geometry_str.substr(comma+1);
        comma = geometry_str.find(" ");
        str = geometry_str.substr(0,comma);
        shp_point.x = atof(str.c_str()) - hdmap_offset.x - m_offset_x;
        geometry_str = geometry_str.substr(comma+1);
        comma = geometry_str.find(")");
        str = geometry_str.substr(0,comma);
        shp_point.y = atof(str.c_str()) - hdmap_offset.y - m_offset_y;
        vec_plane_points.push_back(shp_point);
        plane_visualization(vec_plane_points);
    }
    plane_marker_pub.publish(plane_array);

    for(int i=0; i<vec_geo_plane.size(); i++)
    {
        // std::cout<<"id : "<<vec_idx[i]<<" st_x: "<<vec_start_x[i]<<" st_y: "<<vec_start_y[i]<<" ed_x: "<<vec_end_x[i]<<" ed_y: "<<vec_end_y[i]<<std::endl;
        geometry_msgs::Point start_point, end_point, centroid_point;
        start_point.x = atof(vec_start_x[i].c_str())- hdmap_offset.x - m_offset_x;;
        start_point.y = atof(vec_start_y[i].c_str())- hdmap_offset.y - m_offset_y;;
        end_point.x = atof(vec_end_x[i].c_str())- hdmap_offset.x - m_offset_x;;
        end_point.y = atof(vec_end_y[i].c_str())- hdmap_offset.y - m_offset_y;;
        centroid_point.x = atof(vec_centroid_x[i].c_str())- hdmap_offset.x - m_offset_x;;
        centroid_point.y = atof(vec_centroid_y[i].c_str())- hdmap_offset.y - m_offset_y;;
        double length = sqrt(pow(end_point.x-start_point.x,2) + pow(end_point.y-start_point.y,2));
        std::cout<<"id : "<<vec_idx[i]<<" st_x: "<<start_point.x<<" st_y: "<<start_point.y<<" ed_x: "<<end_point.x<<" ed_y: "<<end_point.y<<" length: "<<length<<std::endl;

        localization_msgs::planar planar_info;
        planar_info.header.frame_id = "odom";
        planar_info.header.stamp = ros::Time::now();
        planar_info.index = i;
        planar_info.start = start_point;
        planar_info.end = end_point;
        planar_info.centroid = centroid_point;
        planar_info.length = length;
        planar_info.theta = atan2(end_point.y-start_point.y, end_point.x-start_point.x)*180/M_PI;
        planars.planar.push_back(planar_info);

        pcl::PointXYZI point;
        point.x = centroid_point.x;
        point.y = centroid_point.y;
        point.z = length;
        point.intensity = planar_info.theta;

        hdmap_planar_points.points.push_back(point);
        // std::cout<<planar_info<<std::endl;
    }
    planars.pc_planar = cvt::cloud2msg(hdmap_planar_points);
    hdmap_planar_pub.publish(planars);

    test_planar_pub.publish(cvt::cloud2msg(hdmap_planar_points, "odom"));

}

void NodeGen::plane_visualization(std::vector<geometry_msgs::Point> vec_points)
{
    static int cnt = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    // marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = cnt++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points.resize(2);
    marker.points[0] = vec_points[0];
    marker.points[1] = vec_points[1];
    plane_array.markers.push_back(marker);
}