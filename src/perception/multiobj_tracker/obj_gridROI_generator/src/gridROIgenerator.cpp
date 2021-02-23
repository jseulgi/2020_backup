#include "gridROIgenerator.h"

ObjGridRoiGenerator::ObjGridRoiGenerator(){
    ROS_INFO_STREAM("init");
    setParams();

    //Publisher
    local_link_map_pub = nh.advertise<sensor_msgs::Image>("/GRIDROI/local_link_map", 1);
    local_lane_map_pub = nh.advertise<sensor_msgs::Image>("/GRIDROI/local_lane_map", 1);
    grid_roi_map_pub   = nh.advertise<sensor_msgs::Image>("/GRIDROI/grid_roi_map", 1);
    link_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/GRIDROI/local_link_pc_pub", 1);
    grid_roi_pub = nh.advertise<obj_msgs::GridRoi>("/GRIDROI/grid_roi_info", 1);

    //Subscriber
    static ros::Subscriber offset_sub = nh.subscribe("/map_info", 1, &ObjGridRoiGenerator::offsetCallback, this);
    static ros::Subscriber gpsodom_sub = nh.subscribe("/gps_odom", 1, &ObjGridRoiGenerator::odomCallback, this);
}

void ObjGridRoiGenerator::setParams(){
    nh.param<std::string>("/GRIDROI/link_shp", link_shp, "LINK.shp");
    nh.param<std::string>("/GRIDROI/link_layer", link_layer, "LINK");
    nh.param<std::string>("/GRIDROI/lane_shp", lane_shp, "LINK.shp");
    nh.param<std::string>("/GRIDROI/lane_layer", lane_layer, "LINK");

    std::cout <<"link_shp  : " << link_shp <<std::endl;
    std::cout <<"link_layer: " << link_layer <<std::endl;
    std::cout <<"lane_shp  : " << lane_shp <<std::endl;
    std::cout <<"lane_layer: " << lane_layer <<std::endl;

    nh.param<int>("/GRIDROI/link_type", shp_link_type, 7);

    nh.param<double>("/GRIDROI/local_map_radius", local_map_radius, 60.0);
    ROS_INFO("local_map_radius: %f", local_map_radius);

    nh.param<double>("/GRIDROI/map_resolution_m", map_resolution, 0.1);
    ROS_INFO("map_resolution: %f", map_resolution);

    nh.param<int>("/GRIDROI/dilate_mask_pixel_size", dil_mask_pixel_size, 20);
    ROS_INFO("dilate_mask_pixel_size: %d", dil_mask_pixel_size);

    nh.param<int>("/GRIDROI/erode_mask_pixel_size", ero_mask_pixel_size, 10);
    ROS_INFO("erode_mask_pixel_size: %d", ero_mask_pixel_size);

    nh.param<int>("/GRIDROI/lane_dilate_mask_pixel_size", lane_dil_mask_pixel_size, 3);
    ROS_INFO("lane_mask_pixel_size: %d", lane_dil_mask_pixel_size);

    nh.param<int>("/GRIDROI/lane_erode_mask_pixel_size", lane_ero_mask_pixel_size, 3);
    ROS_INFO("lane_mask_pixel_size: %d", lane_ero_mask_pixel_size);
}

// Get the reference global point from global map as a offset.
void ObjGridRoiGenerator::offsetCallback(const path_msgs::Map& offset_msg){
    //Run once
    ROS_INFO_STREAM("Callback offset");
    if(!is_offsetVal){
        ROS_INFO_STREAM("offsetMap callback");
        mapX_offset = offset_msg.OffsetMapX;
        mapY_offset = offset_msg.OffsetMapY;
        OpenLinkSHP();
        OpenLaneSHP();
        is_offsetVal = true;
    }
    return;
}

// Main callback
void ObjGridRoiGenerator::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    // ROS_INFO_STREAM("Callback odom");
    if (!is_offsetVal){
        ROS_WARN_STREAM("Not Map initialized Yet!");
        return;
    }

    std::clock_t mapping_s = std::clock();
    geometry_msgs::Pose ego_pose;
    ego_pose.position.x = odom_msg->pose.pose.position.x ;
    ego_pose.position.y = odom_msg->pose.pose.position.y ;
    ego_pose.position.z = 0;

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (is_first_odom){
        is_first_odom = false;
        mapYaw_offset = yaw;
    }
    else{
        diff_heading = yaw - mapYaw_offset;
    }
    if (diff_heading >= 2*M_PI){
        diff_heading -= 2*M_PI;
    }
    if (diff_heading <= -2*M_PI){
        diff_heading += 2*M_PI;
    }
    genGridROI(ego_pose.position.x, ego_pose.position.y, diff_heading);

    std::clock_t mapping_e = std::clock();
    // std::cout << "Mapping Running Time  : " << static_cast<double>((mapping_e-mapping_s))/CLOCKS_PER_SEC << std::endl;
    return;
}


void ObjGridRoiGenerator::genGridROI(double egoGlobal_x, double egoGlobal_y, double egoGlobal_yaw){
    std::clock_t time_s, time_e;
    int map_size = 2*local_map_radius;
    int img_rows = int(map_size / map_resolution);
    int img_cols = int(map_size / map_resolution);
    int center_row = img_rows/2;
    int center_col = img_cols/2;

    cv::Mat roi_img = cv::Mat(img_rows, img_cols, CV_8UC1, cv::Scalar::all(255));
    cv::Mat lane_img = cv::Mat(img_rows, img_cols, CV_8UC1, cv::Scalar::all(255));

    time_s = std::clock();
    std::vector<int> local_link_indices;
    Kdtree(globalLink_pc, egoGlobal_x, egoGlobal_y, local_link_indices);
    PointT ego_link_point = globalLink_pc.points[local_link_indices[0]];

    std::vector<int> local_lane_indices;
    Kdtree(globalLane_pc, egoGlobal_x, egoGlobal_y, local_lane_indices);
    time_e = std::clock();
    // switch(int(ego_link_point.intensity)){
    //     case 100:
    //         std::cout << "\033[1;33mon INTERSECTION\033[0m" << std::endl;
    //         break;
    //     case 50:
    //         std::cout << "\033[1;33mon CLOCK_WISE_ROAD\033[0m" << std::endl;
    //         break;
    //     case 150:
    //         std::cout << "\033[1;33mon COUNTER_CLOCK_WISE_ROAD\033[0m" << std::endl;
    //         break;
    //     default:
    //         std::cout << "\033[1;33mon WHERE AM I\033[0m" << std::endl;
    //         break;
    // }
    // std::cout << "kdtree Running Time  : " << static_cast<double>((time_e-time_s))/CLOCKS_PER_SEC << std::endl;

    time_s = std::clock();
    pcl::PointCloud<PointT>::Ptr localLink_pc(new pcl::PointCloud<PointT>);
    // std::cout << "local link point num: " <<local_link_indices.size() <<std::endl;
    for (int searchIdx = 0; searchIdx < local_link_indices.size(); searchIdx++){
        PointT global_pnt;
        PointT local_pnt;

        global_pnt = globalLink_pc.points[local_link_indices[searchIdx]];
        local_pnt.x = (global_pnt.x - egoGlobal_x) * cos(-(egoGlobal_yaw + mapYaw_offset)) - (global_pnt.y - egoGlobal_y) * sin(-(egoGlobal_yaw + mapYaw_offset));
        local_pnt.y = (global_pnt.x - egoGlobal_x) * sin(-(egoGlobal_yaw + mapYaw_offset)) + (global_pnt.y - egoGlobal_y) * cos(-(egoGlobal_yaw + mapYaw_offset));
        local_pnt.z = 0;
        // local_pnt.x = (global_pnt.x - egoGlobal_x) * cos(-mapYaw_offset) - (global_pnt.y - egoGlobal_y) * sin(-mapYaw_offset);
        // local_pnt.y = (global_pnt.x - egoGlobal_x) * sin(-mapYaw_offset) + (global_pnt.y - egoGlobal_y) * cos(-mapYaw_offset);
        // local_pnt.z = 0;
        local_pnt.intensity = global_pnt.intensity;
        localLink_pc->points.push_back(local_pnt);

        int img_row = int(center_row - local_pnt.x / map_resolution );
        int img_col = int(center_col - local_pnt.y / map_resolution );
        roi_img.at<int8_t>(img_row, img_col) = 0;
    }

    pcl::PointCloud<PointT>::Ptr locallane_pc(new pcl::PointCloud<PointT>);
    // std::cout << "local lane point num: " <<local_lane_indices.size() <<std::endl;
    for (int searchIdx = 0; searchIdx < local_lane_indices.size(); searchIdx++){
        PointT global_pnt;
        PointT local_pnt;

        global_pnt = globalLane_pc.points[local_lane_indices[searchIdx]];
        local_pnt.x = (global_pnt.x - egoGlobal_x) * cos(-(egoGlobal_yaw + mapYaw_offset)) - (global_pnt.y - egoGlobal_y) * sin(-(egoGlobal_yaw + mapYaw_offset));
        local_pnt.y = (global_pnt.x - egoGlobal_x) * sin(-(egoGlobal_yaw + mapYaw_offset)) + (global_pnt.y - egoGlobal_y) * cos(-(egoGlobal_yaw + mapYaw_offset));
        local_pnt.z = 0;
        locallane_pc->points.push_back(local_pnt);

        int img_row = int(center_row - local_pnt.x / map_resolution );
        int img_col = int(center_col - local_pnt.y / map_resolution );
        lane_img.at<int8_t>(img_row, img_col) = 0;
    }
    time_e = std::clock();
    // std::cout << "for loop Running Time  : " << static_cast<double>((time_e-time_s))/CLOCKS_PER_SEC << std::endl;

    time_s = std::clock();

    cv::Mat canny_img;
    cv::Mat dilation_img;
    cv::Mat local_link_map;

    cv::Mat lane_canny_img;
    cv::Mat lane_dilation_img;
    cv::Mat lane_elosion_img;
    cv::Mat local_lane_img;

    cv::Mat grid_roi_map;

    cv::Mat link_dil_mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dil_mask_pixel_size,dil_mask_pixel_size), cv::Point(1,1));
    cv::Mat link_ero_mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ero_mask_pixel_size,ero_mask_pixel_size), cv::Point(1,1));
    cv::Mat lane_dil_mask= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lane_dil_mask_pixel_size,lane_dil_mask_pixel_size*2), cv::Point(1,1));
    cv::Mat lane_ero_mask= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lane_ero_mask_pixel_size,lane_ero_mask_pixel_size), cv::Point(1,1));

    cv::Canny(roi_img, canny_img, 50, 200, 3);
    cv::Canny(lane_img,lane_canny_img, 50, 200, 3);

    cv::dilate(lane_canny_img, lane_dilation_img, lane_dil_mask, cv::Point(-1,-1), 1);
    cv::erode(lane_dilation_img, lane_elosion_img, lane_ero_mask, cv::Point(-1,-1), 1);
    cv::bitwise_not(lane_elosion_img, local_lane_img);

    cv::dilate(canny_img, dilation_img, link_dil_mask, cv::Point(-1, -1), 1);
    cv::erode(dilation_img, local_link_map, link_ero_mask, cv::Point(-1, -1), 1);

    cv::bitwise_and(dilation_img, local_lane_img, grid_roi_map);

    time_e = std::clock();
    // std::cout << "cv function Running Time  : " << static_cast<double>((time_e-time_s))/CLOCKS_PER_SEC << std::endl;

    /////////////////////////////
    // publish to obj_detector //
    /////////////////////////////

    cv_bridge::CvImage lane_img_msg;
    lane_img_msg.header.frame_id = "map";
    lane_img_msg.header.stamp = ros::Time::now();
    lane_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    lane_img_msg.image = local_lane_img;
    local_lane_map_pub.publish(lane_img_msg.toImageMsg());

    cv_bridge::CvImage link_img_msg;
    link_img_msg.header.frame_id = "map";
    link_img_msg.header.stamp = ros::Time::now();
    link_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    link_img_msg.image = local_link_map;
    local_link_map_pub.publish(link_img_msg.toImageMsg());

    cv_bridge::CvImage out_img_msg;
    out_img_msg.header.frame_id = "map";
    out_img_msg.header.stamp = ros::Time::now();
    out_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_img_msg.image = grid_roi_map;
    grid_roi_map_pub.publish(out_img_msg.toImageMsg());

    sensor_msgs::PointCloud2 local_link_pnts_msg;
    localLink_pc->header.frame_id = "base_link";
    pcl_conversions::toPCL(ros::Time::now(), localLink_pc->header.stamp);
    localLink_pc->width = localLink_pc->size();
    localLink_pc->height = 1;
    localLink_pc->is_dense = true;
    pcl::toROSMsg(*localLink_pc, local_link_pnts_msg);
    link_pc_pub.publish(local_link_pnts_msg);

    obj_msgs::GridRoi grid_roi_msg;
    grid_roi_msg.stamp = ros::Time::now();
    grid_roi_msg.map_resolution = map_resolution;           // published image resolution
    grid_roi_msg.ego_link_point.x = ego_link_point.x - egoGlobal_x;
    grid_roi_msg.ego_link_point.y = ego_link_point.y - egoGlobal_y;
    grid_roi_msg.ego_link_point.z = ego_link_point.z - 0;
    grid_roi_msg.global_yaw_ref_map = egoGlobal_yaw;
    grid_roi_msg.ego_link_type = ego_link_point.intensity;
    grid_roi_msg.roi_value = 255;                             // the value meaning that the cell is on ROI
    grid_roi_msg.grid_roi = *(out_img_msg.toImageMsg());    //published image meaning local ROI
    grid_roi_msg.local_link_points = local_link_pnts_msg;   //the points of surrounding link
    grid_roi_pub.publish(grid_roi_msg);

    return;
}


void ObjGridRoiGenerator::Kdtree(pcl::PointCloud<PointT> &src_pc, float x_search, float y_search, std::vector<int> &searched_indices){
    pcl::KdTreeFLANN<PointT> kdtree;
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    PointT k_searchPoint;
    k_searchPoint.x = x_search;
    k_searchPoint.y = y_search;
    k_searchPoint.z = 0;
    pcl::PointCloud<PointT>::Ptr input_pc (new pcl::PointCloud<PointT>);
    *input_pc = src_pc;
    kdtree.setInputCloud(input_pc);
    kdtree.radiusSearch(k_searchPoint, local_map_radius, k_indices, k_distances);
    searched_indices = k_indices;
}

void ObjGridRoiGenerator::OpenLinkSHP(){
    PointT hdmap_link_points;

    OGRRegisterAll();
    // std::vector<geometry_msgs::Point> vector_link_lane;

    // link_shp = "/home/urlautocar/kakauto_ws/src/autonomous-car/map_generation/HD_MAP/HD_MAP_alpha_final/A3_LINK_V5.shp";
    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx( link_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL ));
    if( poDS == NULL )
    {
        printf( "Open failed.\n" );
        exit( 1 );
    }

    std::cout << "\033[1;33mHD LINK Open Successed\033[0m" << std::endl;
    OGRLayer  *poLayer = poDS->GetLayerByName( link_layer.c_str() );
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;
    int link_pnt_num=0;
    int target_link_lane = std::numeric_limits<int>::max();
    while( (poFeature = poLayer->GetNextFeature()) != NULL ){
        for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ ){
            // cout <<"percent done:"<<iField<<"/"<<poFDefn->GetFieldCount() << endl;
            OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
            std::string str = "";
            str = ParsingToString( poFieldDefn, poFeature, iField);
            unsigned long comma;
            std::string str_temp;
            // cout << str << endl;
            if ( iField == shp_link_type ){
                // std::cout << atoi(str.c_str())<<std::endl;
                target_link_lane = atoi(str.c_str());
                // std::cout<<target_link_lane<<std::endl;
                // vector_link_lane.push_back(target_link_lane);
            }
        }

        char *pszWKT = NULL;
        std::string geometry_str;
        std::vector<geometry_msgs::Point> link_array;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString ) {
            poGeometry->exportToWkt(&pszWKT);
            geometry_str = pszWKT;
            //printf("%s\n", pszWKT);
            CPLFree(pszWKT);
        }
        else {
            printf( "LINK : no point geometry\n" );}

        link_array = parsing_link_geometry(geometry_str);
        link_pnt_num += link_array.size();
        for(int i=0; i<link_array.size(); i++){

            hdmap_link_points.x = link_array[i].x - mapX_offset;
            hdmap_link_points.y = link_array[i].y - mapY_offset;
            hdmap_link_points.z = 0;

            switch(target_link_lane){
                case INTERSECTION:
                    hdmap_link_points.intensity = 100;
                    break;
                case CLOCK_WISE:
                    hdmap_link_points.intensity = 50;
                    break;
                case COUNTER_CLOCK_WISE:
                    hdmap_link_points.intensity = 150;
                    break;
                default:
                    hdmap_link_points.intensity = 100;
                    break;
            }
            globalLink_pc.points.push_back(hdmap_link_points);
        }
        OGRFeature::DestroyFeature( poFeature );
    }
    ROS_INFO_STREAM("Total link points: "+ std::to_string(link_pnt_num));
    GDALClose( poDS );
}

std::vector<geometry_msgs::Point> ObjGridRoiGenerator::parsing_link_geometry(std::string geometry_str){
    std::string str;
    unsigned long comma;
    unsigned long space;

    geometry_msgs::Point point;
    std::vector<geometry_msgs::Point> point_array;

    comma = geometry_str.find("(");
    geometry_str = geometry_str.substr(comma+1);

    while(geometry_str.find(",") != std::string::npos){
        comma = geometry_str.find(",");
        str = geometry_str.substr(0,comma);
        space = str.find(" ");
        point.x = atof(str.substr(0,space).c_str());
        str = str.substr(space+1);
        space = str.find(" ");
        point.y = atof(str.substr(0,space).c_str());
        point_array.push_back(point);
        geometry_str = geometry_str.substr(comma+1);
    }

    space = geometry_str.find(" ");
    point.x = atof(geometry_str.substr(0,space).c_str());
    geometry_str = geometry_str.substr(space+1);
    space = geometry_str.find(" ");
    point.y = atof(geometry_str.substr(0,space).c_str());
    point_array.push_back(point);

    return point_array;
}

void ObjGridRoiGenerator::OpenLaneSHP(){

    PointT hdmap_lane_points;
    std::vector<geometry_msgs::Point> LaneGeoVector;

    OGRRegisterAll();
    GDALDataset *poDS = static_cast<GDALDataset*>(GDALOpenEx(lane_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL));

    if( poDS == NULL ){
        printf( "Open lane failed.\n" );
        exit( 1 );}

    std::cout << "\033[1;33mHD LANE Open Successed\033[0m" << std::endl;

    OGRLayer *poLayer = poDS->GetLayerByName(lane_layer.c_str());
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;
    int lane_pnt_num=0;
    while( (poFeature = poLayer->GetNextFeature()) != NULL ){
        char *pszWKT = NULL;
        std::string geometry_str;
        std::vector<geometry_msgs::Point> lane_array;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString ){
            poGeometry->exportToWkt(&pszWKT);
            geometry_str = pszWKT;
            // printf("%s\n", pszWKT);
            CPLFree(pszWKT);
        }
        else{
            printf( "LANE : no point geometry\n" );}

        // std::cout << geometry_str << std::endl;
        lane_array = ParsingLaneGeometry(geometry_str);
        lane_pnt_num += lane_array.size();
        for(int i = 0; i < lane_array.size(); i++)
        {
            hdmap_lane_points.x = lane_array[i].x - mapX_offset;
            hdmap_lane_points.y = lane_array[i].y - mapY_offset;
            hdmap_lane_points.z = 0;

            globalLane_pc.points.push_back(hdmap_lane_points);
        }
        // ROS_INFO_STREAM("Total link points: "+ std::to_string(lane_pnt_num));
        OGRFeature::DestroyFeature( poFeature );

    }
    GDALClose( poDS );
}

std::vector<geometry_msgs::Point> ObjGridRoiGenerator::ParsingLaneGeometry(std::string geometry_str){
    std::string str;
    unsigned long comma;
    unsigned long space;

    geometry_msgs::Point lane_point;
    std::vector<geometry_msgs::Point> lane_point_array;

    comma = geometry_str.find("(");
    geometry_str = geometry_str.substr(comma+1);
    comma = geometry_str.find(" ");
    str = geometry_str.substr(0,comma);
    geometry_str = geometry_str.substr(comma+1);
    comma = geometry_str.find(" ");
    str = geometry_str.substr(0,comma);
    comma = geometry_str.find(",");
    geometry_str = geometry_str.substr(comma+1);

    while(geometry_str.find(",") != std::string::npos){
        comma = geometry_str.find(",");
        str = geometry_str.substr(0,comma);
        space = str.find(" ");
        lane_point.x = atof(str.substr(0,space).c_str());
        str = str.substr(space+1);
        space = str.find(" ");
        lane_point.y = atof(str.substr(0,space).c_str());
        lane_point.z = 0;
        lane_point_array.push_back(lane_point);
        geometry_str = geometry_str.substr(comma+1);
    }
    space = geometry_str.find(" ");
    lane_point.x = atof(geometry_str.substr(0,space).c_str()) ;
    geometry_str = geometry_str.substr(space+1);
    space = geometry_str.find(" ");
    lane_point.y = atof(geometry_str.substr(0,space).c_str());
    lane_point.z = 0;
    lane_point_array.push_back(lane_point);

    return lane_point_array;
}

std::string ObjGridRoiGenerator::ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField){
    std::string str = "";
    switch( poFieldDefn->GetType() )
    {
        case OFTInteger:
            // printf( "(%d %d)", poFeature->GetFieldAsInteger( iField ), iField );
            str.append( std::to_string( poFeature->GetFieldAsInteger(iField) ) );
            break;
        case OFTInteger64:
            // printf( CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64( iField ) );
            str.append( std::to_string( poFeature->GetFieldAsInteger64(iField) ) );
            break;
        case OFTReal:
            // printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
            str.append( std::to_string( poFeature->GetFieldAsDouble(iField) ) );
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
