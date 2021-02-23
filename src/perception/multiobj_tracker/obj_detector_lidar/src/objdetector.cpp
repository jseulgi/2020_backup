#include "objdetector.h"

bool point_Zcmp(PointT a, PointT b){
    return a.z<b.z;
}
bool point_Xcmp(PointT a, PointT b){
    return a.x<b.x;
}

ObjDetectorLidar::ObjDetectorLidar(){
    setParams();

    //Subscriber
    static ros::Subscriber ego_state_sub = nh.subscribe ("/vehicle_state", 1, &ObjDetectorLidar::egoStateCallback, this);
    static ros::Subscriber grid_roi_sub = nh.subscribe("/GRIDROI/grid_roi_info", 1, &ObjDetectorLidar::gridroiCallback, this);
    static ros::Subscriber cloud_sub = nh.subscribe ("os_cloud_node/points", 1, &ObjDetectorLidar::cloudCallback,this);

    //Publisher
    origin_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/origin_cloud", 1);
    prefiltered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/prefiltered_cloud", 1);

    ground_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/ground_cloud", 1);
    ground_nonroi_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/ground_nonroi_cloud", 1);
    object_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/object_cloud", 1);
    obj_highint_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/obj_high_intensity", 1);
    object_nonroi_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("detector/object_nonroi_cloud", 1);

    intensity_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("detector/intensity_markers", 1);
    obj_boundaries_pub = nh.advertise<obj_msgs::ObjTrackBoxes>("detector/detected_bboxes", 1);
}

void ObjDetectorLidar::setParams(){
    nh.param<std::string>("/Sensor/model", sensor_model, "OS2");
    nh.param<double>("/Sensor/distance_interval", distance_interval, 10);
    // if (sensor_model == "OS1"){vertical_res = 0.7;}
    // tolerance_interval = distance_interval * 2.0 * tan(vertical_res/2.0);
    nh.param<bool>("/ROI/use_GridROI", use_gridroi, true);
    nh.param<double>("/ROI/pc_top",    ROI_Top,     0.0);
    nh.param<double>("/ROI/pc_bottom", ROI_Bottom, -2.5);
    nh.param<double>("/ROI/pc_left",   ROI_Left,   20.0);
    nh.param<double>("/ROI/pc_right",  ROI_Right, -20.0);
    nh.param<double>("/ROI/pc_front",  ROI_Front,  60.0);
    nh.param<double>("/ROI/pc_back",   ROI_Back,  -30.0);
    ROI_maxpoint =Eigen::Vector4f( ROI_Front, ROI_Left,   ROI_Top,     1.0f);
    ROI_minpoint =Eigen::Vector4f( ROI_Back,  ROI_Right,  ROI_Bottom,  1.0f);

    nh.param<double>("/CloudPrefilter/voxel_size", voxel_grid_size, 0.1);
    nh.param<int>("/CloudPrefilter/voxel_minimum_size", voxel_minimum_size, 2);
    nh.param<double>("/CloudPrefilter/intensity_thread", intensity_thread, 300.0);

    nh.param<int>("/Ground/num_seg", num_seg, 1);
    nh.param<int>("/Ground/num_iter",groundext_iter_num, 1);
    nh.param<int>("/Ground/num_lpr", num_lpr, 20);
    nh.param<double>("/Ground/seed_Hthd",seed_Hthd, 1.2);
    nh.param<double>("/Ground/plane_Dthd", plane_Dthd, 0.3);

    nh.param<int>("/Clustering/clustering_min_size", ec_minpnt_size, 10);
    nh.param<int>("/Clustering/clustering_max_size", ec_maxpnt_size, 1200);
    nh.param<double>("/Clustering/euclidean_initial_tolerance", init_tolerance, 0.20);

    nh.param<bool>("/SizeFilter/use_size_filter", use_size_filter, false);
    nh.param<double>("/SizeFilter/X_maximum_scale", obj_maxXscale, 10.0);
    nh.param<double>("/SizeFilter/X_minimum_scale", obj_minXscale, 0.5);
    nh.param<double>("/SizeFilter/Y_maximum_scale", obj_maxYscale, 10.0);
    nh.param<double>("/SizeFilter/Y_minimum_scale", obj_minYscale, 0.5);
    nh.param<double>("/SizeFilter/Z_minimum_scale", obj_minZscale, 0.5);
    nh.param<bool>("/SizeFilter/use_intensity_filter", use_intensity_filter, true);
    nh.param<double>("/SizeFilter/high_intensity_ratio_thread", highI_rThd, 0.3);
    nh.param<bool>("/SizeFilter/use_iou_combine", use_iou_combine, true);

    nh.param<double>("/Ego/width", egoWidth, 1.795);
    nh.param<double>("/Ego/length", egoLength, 4.300);
}

// get ego speed from can data_
void ObjDetectorLidar::egoStateCallback(const control_msgs::VehicleState &can_bus_msg){
    ego_speed = can_bus_msg.v_ego;
    ego_yaw_rate = can_bus_msg.yaw_rate;
    return;
}

// use GridROI
void ObjDetectorLidar::gridroiCallback(const obj_msgs::GridRoi &gridroi_msg){
    temp_gridroi_msg = gridroi_msg;
    is_gridroi = true;
    return;
}

void ObjDetectorLidar::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &sensor_pc){
    // std::cout << "=====================Start Line=====================" << std::endl;
    std::clock_t start_cb = std::clock();

    ///////////////////////////////////////
    ////    ====Initialize Msgs====    ////
    ///////////////////////////////////////
    //Initialize gridroi_msg
    if (!is_gridroi){
        ROS_WARN_STREAM("Do not have gridroi_msg yet");
        return;
    }
    obj_msgs::GridRoi input_grid_roi_msg = temp_gridroi_msg;
    // std::cout <<"Time Interval between callbacks: " <<ros::Time::now() - input_grid_roi_msg.stamp<< std::endl;

    gridroi_stamp = input_grid_roi_msg.stamp;
    grid_roi_resolution = input_grid_roi_msg.map_resolution;
    ego_link_point = input_grid_roi_msg.ego_link_point;
    ego_link_type = input_grid_roi_msg.ego_link_type;
    global_yaw_ref_map = input_grid_roi_msg.global_yaw_ref_map;
    roi_value = input_grid_roi_msg.roi_value;

    cv_bridge::CvImagePtr gridroi_img;
    gridroi_img = cv_bridge::toCvCopy(input_grid_roi_msg.grid_roi, sensor_msgs::image_encodings::TYPE_8UC1);
    local_grid_roi = gridroi_img->image;
    link_points = input_grid_roi_msg.local_link_points;

    //Initialize can_bus_msg
    cur_speed = ego_speed / 3.6 ;
    cur_yaw_rate = ego_yaw_rate;

    //Initialize pointcloud msg
    pcl::PointCloud<PointT>::Ptr src_cloud(new pcl::PointCloud<PointT>);
    // sensor_msgs::PointCloud2 input = *sensor_pc;
    // input.header.stamp = ros::Time::now();
    // pcl::fromROSMsg(input, *src_cloud);
    pcl::fromROSMsg(*sensor_pc, *src_cloud);
    src_cloud->header.frame_id = "base_link";
    src_cloud->header.stamp = src_cloud->header.stamp;
    src_cloud->width = src_cloud->size();
    src_cloud->height = 1;
    src_cloud->is_dense = true;
    if (src_cloud->size() == 0){
        ROS_WARN_STREAM("No Point in Cloud");
        is_gridroi = false;
        return;
    }

    ///////////////////////////////////////
    ////    ====Cloud Transform====    ////
    /////////////////////////////////////// //This step is only needed to OS1
    // tf::Transform transform;
    // transform.setRotation(tf::Quaternion( 0.0047996, 0, 0.9999885, 0));
    // transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    // pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>);
    // pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
    // transformed->header.frame_id = "base_link";
    // transformed->header.stamp = src_cloud->header.stamp;
    // transformed->width = transformed->size();
    // transformed->height = 1;
    // transformed->is_dense = true;

    std::clock_t start_time;
    std::clock_t end_time;

    ///////////////////////////////////////
    ////    ====Cloud Prefilter====    ////
    ///////////////////////////////////////
    start_time = std::clock();
    pcl::PointCloud<PointT>::Ptr prefiltered_pc(new pcl::PointCloud<PointT>);
    cloudPrefilter(src_cloud, prefiltered_pc);
    end_time = std::clock();
    // std::cout << "Running Time of Prefilter  : " << static_cast<double>((end_time-start_time))/CLOCKS_PER_SEC << std::endl;

    /////////////////////////////////////////
    ////    ====Object Clustering====    ////
    /////////////////////////////////////////
    start_time = std::clock();
    pcl::PointCloud<PointT>::Ptr clusteredObjs_pc(new pcl::PointCloud<PointT>);
    objectCluster(prefiltered_pc, clusteredObjs_pc);
    end_time = std::clock();
    // std::cout << "Running Time of Clustering : " << static_cast<double>((end_time-start_time))/CLOCKS_PER_SEC << std::endl;

    /////////////////////////////////////////
    ////    ====Publish Detection====    ////
    /////////////////////////////////////////
    // Origin_pc: Transformed pc which is base cloud before prefitlering
    sensor_msgs::PointCloud2 origin_pc_msg;
    pcl::toROSMsg(*src_cloud, origin_pc_msg);
    origin_pc_pub.publish(origin_pc_msg);

    // Prefiltered_pc: Pointcloud after CropROI, Road Extraction,
    sensor_msgs::PointCloud2 prefiltered_pc_msg;
    pcl::toROSMsg(*prefiltered_pc, prefiltered_pc_msg);
    prefiltered_pc_pub.publish(prefiltered_pc_msg);

    // Object pc pub: Point in this Pointcloiud is a member of clustered points
    sensor_msgs::PointCloud2 obj_pc_msg;
    pcl::toROSMsg(*clusteredObjs_pc, obj_pc_msg);
    object_pc_pub.publish(obj_pc_msg);

    std::clock_t end_cb = std::clock();
    is_gridroi = false;

    // std::cout << "Detector Running Time  : " << static_cast<double>((end_cb-start_cb))/CLOCKS_PER_SEC << std::endl;
    return;
}


//Brief: Main Function for prefiltering
//
//Input:  pcl::PointCloud<PointT> :  original point cloud
//Output: pcl::PointCloud<PointT> :  prefiltered point cloud
void ObjDetectorLidar::cloudPrefilter(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc){
    pcl::PointCloud<PointT>::Ptr filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr croped (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr voxelized (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr nongnd (new pcl::PointCloud<PointT>);

    Crop3dROI(input_pc, croped, ROI_maxpoint, ROI_minpoint);

    if (input_pc->size() == 0){
        ROS_WARN_STREAM("No Point in ROI");
        return ;
    }

    Voxelizer(croped, voxelized, voxel_grid_size);

    RoadEliminator(voxelized, nongnd);

    *output_pc = *nongnd;
    output_pc->header = input_pc->header;
    output_pc->width = output_pc->size();
    output_pc->height = 1;
    output_pc->is_dense = true;
}


//Brief:  Return the points within Region of Interest
//
//Input:  pcl::PointCloud<PointT> :  original point cloud,
//        Eigen::Vector4f : max point (x_max, y_max, z_max, 1) of ROI Cube
//        Eigen::Vector4f : min point (x_min, y_min, z_min, 1) of ROI Cube
//Output: pcl::PointCloud<PointT> : point cloud within ROI
void ObjDetectorLidar::Crop3dROI(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc, Eigen::Vector4f Cubepoint_max, Eigen::Vector4f Cubepoint_min){
    pcl::PointCloud<PointT> filtered;
    pcl::CropBox<PointT> boxfilter;
    boxfilter.setMin(Cubepoint_min);
    boxfilter.setMax(Cubepoint_max);
    boxfilter.setInputCloud(input_pc);
    boxfilter.filter(filtered);
    *output_pc = filtered;
    output_pc->header = input_pc->header;
}


//Brief:  Set the resolution of point cloud (Downsampling)
//
//Input:  pcl::PointCloud<PointT> :  original point cloud,
//        double                  :  target resolution of point cloud
//Output: pcl::PointCloud<PointT> : point cloud within ROI
void ObjDetectorLidar::Voxelizer(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc, double leafsize){
    pcl::PointCloud<PointT> filtered;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (input_pc);
    vg.setMinimumPointsNumberPerVoxel(voxel_minimum_size);
    vg.setLeafSize (leafsize, leafsize, leafsize);
    vg.filter (filtered);
    *output_pc = filtered;
    output_pc->header = input_pc->header;
}


//Brief: Return the point which is not included in ground, and Publish the points meaning ground
//
//Input  :  pcl::PointCloud<PointT> :  original point cloud,
//Output :  pcl::PointCloud<PointT> :  point cloud within ROI
//Publish:  sensor_msgs::PointCloud2:  publish pointcloud meaning road(or ground)
void ObjDetectorLidar::RoadEliminator(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc){
    pcl::PointCloud<PointT>::Ptr origin_pc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr sorted_pc(new pcl::PointCloud<PointT>);
    *origin_pc = *input_pc;
    *sorted_pc = *input_pc;
    sort(sorted_pc->points.begin(), sorted_pc->points.end(), point_Zcmp);

    pcl::PointCloud<PointT>::Ptr ground_seeds(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr ground_pc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr ground_nonroi_pc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr object_pc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr obj_highint_pc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr object_nonroi_pc(new pcl::PointCloud<PointT>);

    std::clock_t time_s, time_e;
    ExtractInitialSeeds(sorted_pc, ground_seeds);

    time_s = std::clock();
    for ( int gxtNum = 0; gxtNum < groundext_iter_num ; gxtNum ++){
        Planefactorizer(ground_seeds);
        ground_seeds->clear();

        Eigen::MatrixXf pntsMatrix(origin_pc->size(), 3);
        int rowCnt = 0;
        for (auto p:origin_pc->points){
            pntsMatrix.row(rowCnt++)<<p.x, p.y, p.z;
        }
        Eigen::VectorXf pnt2plane = pntsMatrix*plane_normal_;

        for (int pNum = 0; pNum < pnt2plane.rows(); pNum++){
            if( gxtNum == groundext_iter_num - 1 ){
                // std::cout << "use_gridroi: " << use_gridROI << std::endl;
                // std::cout << "is_gridroi  " << is_gridroi << std::endl;
                if(use_gridroi){
                    bool pnt_gridroi = in_GridROI(origin_pc->points[pNum], roi_value);
                    if(pnt_gridroi){
                    // IN GRID ROI
                        if( pnt2plane[pNum] < ground_Dthd ){ ground_pc->points.push_back(origin_pc->points[pNum]); }
                        else{
                            if (origin_pc->points[pNum].intensity > intensity_thread){ obj_highint_pc->points.push_back(origin_pc->points[pNum]);}
                            object_pc->points.push_back(origin_pc->points[pNum]);
                        }
                    }
                    else{
                    // Not In GRID ROI
                        if( pnt2plane[pNum] < ground_Dthd ){ ground_nonroi_pc->points.push_back(origin_pc->points[pNum]); }
                        else{ object_nonroi_pc->points.push_back(origin_pc->points[pNum]); }
                    }
                }
                else{
                    ROS_INFO_STREAM("Do Not Use GridROI");
                    if( pnt2plane[pNum] < ground_Dthd ){ ground_pc->points.push_back(origin_pc->points[pNum]); }
                    else{ object_pc->points.push_back(origin_pc->points[pNum]); }
                }
            }
            else{
                // ROS_WARN_STREAM("FUCK2");
                if( pnt2plane[pNum] < ground_Dthd ){
                    ground_seeds->points.push_back(origin_pc->points[pNum]);
                }
            }
        }
    }
    time_e = std::clock();
    // std::cout << "Running Time of PointCloud separating : " << static_cast<double>((time_e-time_s))/CLOCKS_PER_SEC << std::endl;

    *output_pc = *object_pc;
    output_pc->header = input_pc->header;


    //Publishing Pointclouds
    obj_highint_pc->header = input_pc->header;
    obj_highint_pc->width = obj_highint_pc->size();
    obj_highint_pc->height = 1;
    obj_highint_pc->is_dense = true;
    sensor_msgs::PointCloud2 obj_highint_pc_msg;
    pcl::toROSMsg(*obj_highint_pc, obj_highint_pc_msg);
    obj_highint_pc_pub.publish(obj_highint_pc_msg);

    object_nonroi_pc->header = input_pc->header;
    object_nonroi_pc->width = object_nonroi_pc->size();
    object_nonroi_pc->height = 1;
    object_nonroi_pc->is_dense = true;
    sensor_msgs::PointCloud2 object_nonroi_pc_msg;
    pcl::toROSMsg(*object_nonroi_pc, object_nonroi_pc_msg);
    object_nonroi_pc_pub.publish(object_nonroi_pc_msg);

    ground_pc->header = input_pc->header;
    ground_pc->width = ground_pc->size();
    ground_pc->height = 1;
    ground_pc->is_dense = true;
    sensor_msgs::PointCloud2 ground_pc_msg;
    pcl::toROSMsg(*ground_pc, ground_pc_msg);
    ground_pc_pub.publish(ground_pc_msg);

    ground_nonroi_pc->header = input_pc->header;
    ground_nonroi_pc->width = ground_nonroi_pc->size();
    ground_nonroi_pc->height = 1;
    ground_nonroi_pc->is_dense = true;
    sensor_msgs::PointCloud2 ground_nonroi_pc_msg;
    pcl::toROSMsg(*ground_nonroi_pc, ground_nonroi_pc_msg);
    ground_nonroi_pc_pub.publish(ground_nonroi_pc_msg);
}


//Brief: Return the point which is not included in ground, and Publish the points meaning ground
//
//Input  :  pcl::PointCloud<PointT> :  pointcloud sorted by z(height),
//Output :  pcl::PointCloud<PointT> :  initial ground points guessed with only z-value
void ObjDetectorLidar::ExtractInitialSeeds(pcl::PointCloud<PointT>::Ptr &sorted_pc, pcl::PointCloud<PointT>::Ptr &output_pc){
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);

    double sum = 0;
    int cnt = 0;
    for (int i=0; i<sorted_pc->size() && cnt<num_lpr ; i++){
        sum += sorted_pc->points[i].z;
        cnt++;
    }
    double lpr_Haverage = cnt!=0?sum/cnt:0;
    filtered->clear();
    for (int pntIdx = 0; pntIdx<sorted_pc->size(); pntIdx++){
        if (sorted_pc->points[pntIdx].z < lpr_Haverage + seed_Hthd ){
            filtered->points.push_back(sorted_pc->points[pntIdx]);
        }
    }
    *output_pc = *filtered;
}

//Brief: SubFunction for RoadEliminator, set parameters for ground plane using Singular Value Decomposition
//
//Input  :  pcl::PointCloud<PointT> :  initially guessed ground points,
//set    :  Eigen::MatrixXf plane_normal_: normal vector of plane from input pointcloud (set a,b,c of "ax + by + cz + d = 0")
//          float plane_d_              : set d of plane model
//          float ground_Dthd            : threshold of distance to ground plane (given plane_Dthd)
void ObjDetectorLidar::Planefactorizer(pcl::PointCloud<PointT>::Ptr &input_pc){
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*input_pc, cov, pc_mean);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    plane_normal_ = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    plane_d = -(plane_normal_.transpose()*seeds_mean)(0,0);
    ground_Dthd = plane_Dthd - plane_d;
}

//Brief : Check that target point in on the GridROI
//
//Input : pcl::PointXYZI : lidar_pnt
//Output: ini            : true_value which is tha value of ROI
bool ObjDetectorLidar::in_GridROI(PointT lidar_pnt, int true_value){
    double pntX = lidar_pnt.x;
    double pntY = lidar_pnt.y;
    int img_cenX = local_grid_roi.rows/2;
    int img_cenY = local_grid_roi.cols/2;
    int img_searchRow = -int(lidar_pnt.x/grid_roi_resolution) + img_cenX;
    int img_searchCol = -int(lidar_pnt.y/grid_roi_resolution) + img_cenY;

    int roi_value = local_grid_roi.at<uchar>(img_searchRow, img_searchCol);

    return roi_value == true_value;
}


//Brief: Return two cloud splited from input cloud
//
//Input  :  pcl::PointCloud<PointT> :  original point cloud,
//Output :  pcl::PointCloud<PointT> :  sub pointcloud 1
//          pcl::PointCloud<PointT> :  sub pointcloud 2
void ObjDetectorLidar::cloudClipper(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc1, pcl::PointCloud<PointT>::Ptr &output_pc2){
    pcl::PointCloud<PointT>::Ptr sorted_pc (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr split_pc1 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr split_pc2 (new pcl::PointCloud<PointT>);
    for (auto it = input_pc->points.begin(); it != input_pc->points.end(); it++){
        if ((*it).y < 0){split_pc1->points.push_back(*it);}
        else            {split_pc2->points.push_back(*it);}
    }
    *output_pc1 = *split_pc1;
    *output_pc2 = *split_pc2;
}


//Brief: Main Function for Clustering
//
//Input:  pcl::PointCloud<PointT> :  input cloud
//Output: pcl::PointCloud<PointT> :  points which are included in objects
void ObjDetectorLidar::objectCluster(pcl::PointCloud<PointT>::Ptr &input_pc, pcl::PointCloud<PointT>::Ptr &output_pc){
    if (input_pc->size() == 0){
        // ROS_WARN_STREAM("No Point to Clustering");

        output_pc->clear();
        output_pc->header = input_pc->header;
        output_pc->width  = output_pc->size();
        output_pc->height = 1;
        output_pc->is_dense = true;

        std::vector<pcl::PointCloud<PointT>> empty_objSet;
        objsPublisher(empty_objSet);

        return;
    }
    std::clock_t start;
    std::clock_t end;

    start = std::clock();

    boost::array<std::vector<int>, num_region> separated_indices;
    regionSeparator(input_pc, separated_indices);

    end = std::clock();
    // std::cout << "Region separator : " << static_cast<double>((end-start))/CLOCKS_PER_SEC << std::endl;

    start = std::clock();
    std::vector<pcl::PointCloud<PointT>> clustered_objSet;
    pcl::PointCloud<PointT>::Ptr clusteredObj_Allpc (new pcl::PointCloud<PointT>);

    //float regionTolerance = init_tolerance;
    float regionTolerance = init_tolerance;
    // ROS_INFO_STREAM(init_tolerance);
    for (int rIdx = 0 ; rIdx < num_region; rIdx++){
        // regionTolerance += tolerance_interval;
        // regionTolerance += 0.13; //callback os1_cloud_node
        regionTolerance += 0.1;    //callback os_cloud_node

        if (separated_indices[rIdx].size() == 0){
            continue;
        }
        if (separated_indices[rIdx].size() < ec_minpnt_size){
            continue;
        }

        std::vector<pcl::PointIndices> clusterd_indicesSet;
        cloudCluster(input_pc, separated_indices[rIdx], regionTolerance, clusterd_indicesSet);

        //change indices in clustered_indices into pointcloud meaning an object
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterd_indicesSet.begin(); it != clusterd_indicesSet.end(); it++){
            pcl::PointCloud<PointT> clustered_obj;

            for (std::vector<int>::const_iterator p_it = it->indices.begin(); p_it != it->indices.end(); p_it++){
                clustered_obj.points.push_back(input_pc->points[*p_it]);
                clusteredObj_Allpc->points.push_back(input_pc->points[*p_it]);
            }

            clustered_obj.header = input_pc->header;
            clustered_obj.width  = clustered_obj.size();
            clustered_obj.height = 1;
            clustered_obj.is_dense = true;
            //clustered_objSet: Set of object pointcloud
            clustered_objSet.push_back(clustered_obj);
        }
    }

    end = std::clock();
    // std::cout << "Clustering : " << static_cast<double>((end-start))/CLOCKS_PER_SEC << std::endl;

    //output: points included in clustered objects
    *output_pc = *clusteredObj_Allpc;
    output_pc->header = input_pc->header;
    output_pc->width  = output_pc->size();
    output_pc->height = 1;
    output_pc->is_dense = true;

    //Object Publsher
    objsPublisher(clustered_objSet);
}


//Brief: Divide the point cloud into (num_region) sets.
//       Points are divided by Eucludean distance to ego
//Input:  pcl::PointCloud<PointT>                    :  input cloud
//Output: boost::array<std::vector<int>, num_region> :  Array of set of points
void ObjDetectorLidar::regionSeparator(pcl::PointCloud<PointT>::Ptr &input_pc, boost::array<std::vector<int>, num_region> &indicesArray_per_region){
    boost::array<std::vector<int>, num_region> indicesArray;
    int count=0;
    for(int pIdx = 0; pIdx < input_pc->size(); pIdx++){
        // input_pc->points[pIdx].z = 0;
        if (abs(input_pc->points[pIdx].x) <= 1.0 && abs(input_pc->points[pIdx].y) <= 2.0) {
            count++;
            continue;
        }

        float D2 = input_pc->points[pIdx].x * input_pc->points[pIdx].x +
                   input_pc->points[pIdx].y * input_pc->points[pIdx].y ;

        float regionFrom = 0;
        for (int rIdx = 0; rIdx < num_region; rIdx++){
            float regionTo = regionFrom + distance_interval;
            if ( ((regionFrom*regionFrom) < D2) && (D2 <= (regionTo * regionTo)) ) {
                indicesArray[rIdx].push_back(pIdx);
                break;
            }
            regionFrom += distance_interval;
        }
    }
    // std::cout << "count: " << count <<std::endl;
    indicesArray_per_region = indicesArray;
}


//Brief: Cluster the given Pointcloud
//Input:  pcl::PointCloud<PointT>        :  input cloud
//Input:  std::vector<int>               :  used indices in input cloud
//Input:  double tolerance               :  tolerance for clustering
//Output: std::vector<pcl::PointIndices> :  Array of indices which are included in a object
void ObjDetectorLidar::cloudCluster(pcl::PointCloud<PointT>::Ptr &input_pc, std::vector<int> &input_indices, double tolerance, std::vector<pcl::PointIndices> &output_indicesSet){
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointIndices> clustered_indices;

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(input_indices));
    tree->setInputCloud(input_pc, indices_array_ptr);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(input_pc);
    ec.setIndices(indices_array_ptr);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(ec_minpnt_size);
    ec.setMaxClusterSize(ec_maxpnt_size);
    ec.extract(clustered_indices);
    output_indicesSet = clustered_indices;
}


//Brief: Publish the boundary boxes and their Markers
//Input  :  std::vector<pcl::PointCloud<PointT>> : Array of clustered objects
//Publish:  obj_msgs::ObjTrackBoxes              : boundary boxes of objects
//          visualization::MarkerArray           : visualization marker for boundary boxes
void ObjDetectorLidar::objsPublisher(std::vector<pcl::PointCloud<PointT>> &clustered_objArray){
    if (clustered_objArray.size() == 0){
        // ROS_WARN_STREAM("PULBISHER ZERO SIZE");
        obj_msgs::ObjTrackBoxes empty_bBoxList;
        empty_bBoxList.header.frame_id = "base_link";
        empty_bBoxList.header.stamp = ros::Time::now();

        empty_bBoxList.vehicle_speed = cur_speed;
        empty_bBoxList.vehicle_yaw_rate = cur_yaw_rate;
        empty_bBoxList.local_link_points = link_points;
        empty_bBoxList.ego_link_point = ego_link_point;
        empty_bBoxList.ego_link_type = ego_link_type;
        empty_bBoxList.global_yaw_ref_map = global_yaw_ref_map;
        empty_bBoxList.bounding_boxes.clear();
        empty_bBoxList.raw_bounding_boxes.clear();
        obj_boundaries_pub.publish(empty_bBoxList);

        return;
    }
    obj_msgs::ObjTrackBoxes bBoxList;
    bBoxList.header = pcl_conversions::fromPCL(clustered_objArray[0].header);
    bBoxList.vehicle_speed = cur_speed;
    bBoxList.vehicle_yaw_rate = cur_yaw_rate;
    bBoxList.local_link_points = link_points;
    bBoxList.ego_link_point = ego_link_point;
    bBoxList.global_yaw_ref_map = global_yaw_ref_map;
    bBoxList.ego_link_type = ego_link_type;

    // visualization_msgs::MarkerArray intensity_markers;

    for ( int objNum = 0 ; objNum < clustered_objArray.size(); objNum++){
        double xmax, ymax, xmin, ymin, zmax, zmin;
        pcl::PointCloud<PointT>::Ptr obj_pc (new pcl::PointCloud<PointT>);
        *obj_pc = clustered_objArray[objNum];
        xmax = obj_pc->points[0].x;
        xmin = obj_pc->points[0].x;
        ymax = obj_pc->points[0].y;
        ymin = obj_pc->points[0].y;
        zmax = obj_pc->points[0].z;
        zmin = obj_pc->points[0].z;

        int high_I_count = 0;
        double avg_intensity = 0;
        for ( auto it = obj_pc->points.begin(); it != obj_pc->points.end(); it++){
            if((*it).x > xmax) { xmax = (*it).x; }
            if((*it).x < xmin) { xmin = (*it).x; }

            if((*it).y > ymax) { ymax = (*it).y; }
            if((*it).y < ymin) { ymin = (*it).y; }

            if((*it).z > zmax) { zmax = (*it).z; }
            if((*it).z < zmin) { zmin = (*it).z; }

            avg_intensity += (*it).intensity;
            if ((*it).intensity > intensity_thread){
                high_I_count++;
            }
        }

        if (sqrt((xmax-xmin)*(xmax-xmin) + (ymax-ymin)*(ymax-ymin)) > 50) {continue;}   //safely detect upto 50m

        avg_intensity /= obj_pc->size();
        // ROS_INFO_STREAM("Point Num " +std::to_string(objNum)+": "+std::to_string(obj_pc->size()));
        // ROS_INFO_STREAM("Intensity " +std::to_string(objNum)+": "+std::to_string(avg_intensity));
        // ROS_INFO_STREAM("ObjHeight " +std::to_string(objNum)+": "+std::to_string(zmax - zmin));
        // Temp visualization to check the average intensities of objects
        // visualization_msgs::Marker text_marker;
        // text_marker.header.frame_id = obj_pc->header.frame_id;
        // text_marker.header.stamp = ros::Time::now();
        // text_marker.id = 1000+objNum;
        // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // text_marker.action = visualization_msgs::Marker::ADD;
        // text_marker.lifetime = ros::Duration(0.2);
        // text_marker.text = "intensity: " + std::to_string(int(avg_intensity));
        // text_marker.pose.orientation.w = 1.0;
        // text_marker.pose.position.x = (xmin+xmax)/2.0;
        // text_marker.pose.position.y = (ymin+ymax)/2.0;
        // text_marker.pose.position.z = 0;
        // text_marker.scale.z = 3.0;
        // text_marker.color.r = 1.0;
        // text_marker.color.g = 1.0;
        // text_marker.color.b = 1.0;
        // text_marker.color.a = 1.0;
        // intensity_markers.markers.push_back(text_marker);
        /////////////////////////////////////////////////////////////////////

        if (use_size_filter){
            if (zmax-zmin < obj_minZscale) {continue;}
            if (xmax-xmin > obj_maxXscale) {continue;}
            if (ymax-ymin > obj_maxYscale) {continue;}
        }

        if (use_intensity_filter){
            double intensity_ratio = ((double)high_I_count)/((double)obj_pc->size());
            // std::cout<< "Intensity ratio: " << intensity_ratio<<std::endl;
            if (intensity_ratio > highI_rThd && zmax < -0.8){continue;}
        }
        // std::cout<< "clusted pc size: "<<obj_pc->size() << std::endl;
        //Publish objects
        obj_msgs::ObjTrackBox bBox;
        bBox.probability = 1;
        bBox.xmin = xmin;
        bBox.ymin = ymin;
        bBox.xmax = xmax;
        bBox.ymax = ymax;

        if (use_iou_combine){
            bool ReviewDone = false;
            if (bBoxList.bounding_boxes.size() > 0){
                for (int boxReview = 0; boxReview < bBoxList.bounding_boxes.size(); boxReview++){
                    double iou = IoUCalculater(bBoxList.bounding_boxes[boxReview], bBox);
                    if (iou!=0.00){
                        bBoxList.bounding_boxes[boxReview].xmax = std::max(bBoxList.bounding_boxes[boxReview].xmax, bBox.xmax);
                        bBoxList.bounding_boxes[boxReview].xmin = std::min(bBoxList.bounding_boxes[boxReview].xmin, bBox.xmin);
                        bBoxList.bounding_boxes[boxReview].ymax = std::max(bBoxList.bounding_boxes[boxReview].ymax, bBox.ymax);
                        bBoxList.bounding_boxes[boxReview].ymin = std::min(bBoxList.bounding_boxes[boxReview].ymin, bBox.ymin);
                        ReviewDone = true;
                        break;
                    }
                }
                if (ReviewDone){continue;}
                else {bBoxList.bounding_boxes.push_back(bBox);}
            }
            else{
                bBoxList.bounding_boxes.push_back(bBox);
            }
        }
        else {
            bBoxList.bounding_boxes.push_back(bBox);
        }
    }

    for (int bBoxNum = 0 ; bBoxNum < bBoxList.bounding_boxes.size(); bBoxNum++) {
        obj_msgs::ObjTrackBox rawBox;
        rawBox.xmax = bBoxList.bounding_boxes[bBoxNum].xmax;
        rawBox.xmin = bBoxList.bounding_boxes[bBoxNum].xmin;
        rawBox.ymax = bBoxList.bounding_boxes[bBoxNum].ymax;
        rawBox.ymin = bBoxList.bounding_boxes[bBoxNum].ymin;
        bBoxList.raw_bounding_boxes.push_back(rawBox);

        // Size up
        bBoxList.bounding_boxes[bBoxNum].xmax += egoLength/2.0;
        bBoxList.bounding_boxes[bBoxNum].xmin -= egoLength/2.0;
        bBoxList.bounding_boxes[bBoxNum].ymax += egoWidth /2.0;
        bBoxList.bounding_boxes[bBoxNum].ymin -= egoWidth /2.0;
    }
    // std::cout << "Number of Detected bBOX: " << bBoxList.bounding_boxes.size() << std::endl;
    // std::cout << "Number of Detected bBOX: " << bBoxList.raw_bounding_boxes.size() << std::endl;
    obj_boundaries_pub.publish(bBoxList);
    // intensity_marker_pub.publish(intensity_markers);
}


//Brief : Compare two bounding boxes by IoU Calculating
//
//Input : obj_msgs::ObjTrackBox  : target_bbox : Existing Boundary in bBoxList
//        obj_msgs::ObjTrackBox  : source_bbox : Newly Given Boundary
//Output: double : IoU score
double ObjDetectorLidar::IoUCalculater(obj_msgs::ObjTrackBox target_bbox, obj_msgs::ObjTrackBox source_bbox){
    double iou_score;
    double targetArea, sourceArea;
    double interArea, unionArea;
    double inter_minX, inter_minY, inter_maxX, inter_maxY;

    inter_minX = std::max(target_bbox.xmin, source_bbox.xmin);
    inter_minY = std::max(target_bbox.ymin, source_bbox.ymin);
    inter_maxX = std::min(target_bbox.xmax, source_bbox.xmax);
    inter_maxY = std::min(target_bbox.ymax, source_bbox.ymax);
    if ( (inter_maxX - inter_minX) <= 0 || (inter_maxY - inter_minY) <= 0 ){
        return 0.0;
    }
    targetArea = (target_bbox.xmax - target_bbox.xmin) * (target_bbox.ymax - target_bbox.ymin);
    sourceArea = (source_bbox.xmax - source_bbox.xmin) * (source_bbox.ymax - source_bbox.ymin);

    interArea  = (inter_maxX - inter_minX) * (inter_maxY - inter_minY);
    unionArea  = sourceArea + targetArea - interArea;
    iou_score = interArea/unionArea;
    return iou_score;
}
