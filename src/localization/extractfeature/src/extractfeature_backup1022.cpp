#include "extractfeature.h"

int regions_[100];

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z<b.z;
}

using namespace unavlib;
feature::feature()
{
    static ros::Subscriber nodegen_sub = m_nh.subscribe<jslocalization::node>("/jslocalization/feature/nodeIn", 10, &feature::node_callback, this);

    xy_clipped_global_pub_ =  m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/xy_clipped_global", 10);
    non_ground_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/non_ground", 1);
    marking_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/road_marking", 1);

    pole_local_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/local", 10);
    test1_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test1", 10);
    test2_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test2", 10);
    test3_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test3", 10);
    test4_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test4", 10);
    test5_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test5", 10);
    test6_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test6", 10);

    pl_raw_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pcl_raw", 1);
    pl_z_clipped_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/z_clipped", 10);
    pl_region_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/clusters", 10);
    pl_2d_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/planar2D", 10);    
    pl_2d_ransac_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/planar2D_ransac", 10);    

    pole_global_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/pole_transformed", 10);
    plane_global_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/plane_transformed", 10);
    pole_obj_pub = m_nh.advertise<jslocalization::ObjTrackBoxes>("/jslocalization/feature/pole_obj", 10);
    plane_obj_pub = m_nh.advertise<jslocalization::planars>("/jslocalization/feature/plane_obj", 10);

    planar_tracking_pub_ = m_nh.advertise<jslocalization::PlanarTrackBoxes>("/jslocalization/feature/planar_trackboxes", 10);
    pole_tracking_pub_ = m_nh.advertise<jslocalization::PoleTrackBoxes>("/jslocalization/feature/pole_trackboxes", 10);
    feature_obj_pub = m_nh.advertise<jslocalization::FeatureBoxes>("/jslocalization/feature/feautre_boxes", 10);


    SetParam();
    AllocateMemory();
}

void feature::AllocateMemory()
{
    pc_pole.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_plane.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_marking.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void feature::ResetParam()
{
    pc_pole->clear();
    pc_plane->clear();
    pc_marking->clear();

}

void feature::SetParam()
{   
    m_nh.param<std::string>("/System/model", sensor_model_, "os1-64"); // VLP-16, HDL-32E, HDL-64E

    // ground
    m_nh.param("/ground/sensor_height", sensor_height_, 2.0);
    m_nh.param("/ground/removal_height", removal_height_, 1.5);
    m_nh.param("/ground/num_iter", num_iter_, 3);
    m_nh.param("/ground/num_lpr", num_lpr_, 20);
    m_nh.param("/ground/th_seeds", th_seeds_, 1.2);
    m_nh.param("/ground/th_dist", th_dist_, 0.3);

    //road
    m_nh.param("/road/max_intensity", max_intensity, 1000.0);
    m_nh.param("/road/min_intensity", min_intensity, 500.0);
    
    // pole
    m_nh.param<float>("/pole/x_clipped", po_x_width, 30.0);
    m_nh.param<float>("/pole/y_clipped", po_y_width, 15.0);
    m_nh.param<float>("/pole/z_axis_min", po_z_axis_min_, -1.5);
    m_nh.param<float>("/pole/z_axis_max", po_z_axis_max_, 7.0);
    m_nh.param<int>("/pole/cluster_size_min", po_cluster_size_min_, 3);
    m_nh.param<int>("/pole/cluster_size_max", po_cluster_size_max_, 700000);
    m_nh.param<float>("/pole/height_pole", po_height, 0.5);
    m_nh.param<float>("/pole/height_max", po_height_max, 10.0);
    m_nh.param<float>("/pole/height_min", po_height_min, 2.0);
    m_nh.param<float>("/pole/voxel_size", po_voxel_size, 0.2);
    m_nh.param<float>("/pole/pole_radius", po_radius, 0.2);
    m_nh.param<float>("/pole/ec_tolerance", po_ec_tolerance, 0.3);
    m_nh.param<float>("/pole/height_trunk", po_trunk_height, 1.0);

    m_nh.param<float>("/planar/z_axis_min", pl_z_axis_min_, -1.0);
    m_nh.param<float>("/planar/z_axis_max", pl_z_axis_max_, 10.0);
    m_nh.param<int>("/planar/cluster_size_min", pl_cluster_size_min_, 5);
    m_nh.param<int>("/planar/cluster_size_max", pl_cluster_size_max_, 700000);
    m_nh.param<float>("/planar/height_plane", pl_height, 2.5);
    m_nh.param<float>("/planar/height_max", pl_height_max, 2.5);
    m_nh.param<float>("/planar/height_min", pl_height_min, 1.0);
    m_nh.param<float>("/planar/ratio_plane", pl_ratio, 0.85);
    m_nh.param<float>("/planar/distance_plane", pl_distance, 2.0);
    m_nh.param<int>("/planar/num_normal_kd", num_normal_kd, 100);
    m_nh.param<float>("/planar/normal_weight", normal_weight, 0.5);
    //region growing 
    m_nh.param<int>("/planar/rg_num_ksearch",rg_num_ksearch,5);
    m_nh.param<int>("/planar/rg_mincluster",rg_mincluster,5);
    m_nh.param<int>("/planar/rg_numneighbor",rg_numneighbor,5);
    m_nh.param<float>("/planar/rg_smoothness",rg_smoothness,5);
    m_nh.param<float>("/planar/rg_curvature",rg_curvature,5);
    m_nh.param<float>("/planar/rs_threshold",ransac_threshold,0.5);
    m_nh.param<int>("/planar/rs_threads",ransac_threads,10);
    m_nh.param<float>("/planar/rs_tolerance",ransac_tolerance,0.5);
    m_nh.param<int>("/planar/rs_min",ransac_min,100);
    m_nh.param<int>("/planar/rs_max",ransac_max,1000);
    m_nh.param<float>("/planar/length_threshold",m_length_threshold,5.0);

    if (sensor_model_ == "os1-64") { // if ec tolerance is 0.1
        regions_[0] = 11; regions_[1] = 11; regions_[2] = 11; regions_[3] = 11; regions_[4] = 11;
        regions_[5] = 11; regions_[6] = 11; regions_[7] = 11; 
    }

}

void feature::node_callback(const jslocalization::node::ConstPtr &node)
{
    Eigen::Matrix4f gps_eigen = cvt::geoPose2eigen(node->gps_odom);
    Eigen::Matrix4f odom_eigen = cvt::geoPose2eigen(node->odom);

    sensor_msgs::PointCloud2::Ptr cloudmsg_in (new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_raw(new pcl::PointCloud<pcl::PointXYZI>);

    *cloudmsg_in = node->lidar;
    pcl::fromROSMsg(*cloudmsg_in, *pcl_pc_raw);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_nonground (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_clipped_x(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_clipped_z (new pcl::PointCloud<pcl::PointXYZI>);

    // 1. remove x side
    remove_side_x(pcl_pc_raw, *pcl_pc_clipped_x, po_x_width); ///50

    // 2. remove y side 
    remove_side_y(pcl_pc_clipped_x, *pcl_pc_in, po_y_width); //50

    // 3. extract road plane
    road_plane_extraction(pcl_pc_in, *pc_nonground);  //pcl_pc_in

    // 4. extract plane object
    extract_plane_object(pcl_pc_raw, gps_eigen, odom_eigen); 

    // 5. extract pole object
    extract_pole_object(pc_nonground, gps_eigen, odom_eigen);

    pcl::PointCloud<pcl::PointXYZI>::Ptr xy_clipped_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_transformed (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*pcl_pc_in, *xy_clipped_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    pcl::transformPointCloud(*pc_plane, *plane_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    pcl::transformPointCloud(*pc_pole, *pole_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    // pcl::transformPointCloud(*pc_nonground, *nonground_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));

    jslocalization::PoleTrackBoxes poleMsg;
    poleMsg.header.frame_id = "odom";
    poleMsg.header.stamp = ros::Time::now();
    poleMsg.gps_odom = node->gps_odom;
    poleMsg.odom = node->odom;
    poleMsg.index = node->index;
    for(int i=0; i<vec_pole.size(); i++)
        poleMsg.objlist.push_back(vec_pole[i]);
    // pole_tracking_pub_.publish(poleMsg);

    jslocalization::PlanarTrackBoxes planarMsg;
    planarMsg.header.frame_id = "odom";
    planarMsg.header.stamp = ros::Time::now();
    planarMsg.gps_odom = node->gps_odom;
    planarMsg.odom = node->odom;
    planarMsg.index = node->index;
    for(int i=0; i<vec_planar.size(); i++)
        planarMsg.objlist.push_back(vec_planar[i]);
    // planar_tracking_pub_.publish(planarMsg);

    jslocalization::FeatureBoxes featureMsg;
    featureMsg.header.frame_id = "odom";
    featureMsg.header.stamp = ros::Time::now();
    featureMsg.gps_odom = node->gps_odom;
    featureMsg.odom = node->odom;
    featureMsg.index = node->index;
    featureMsg.planarBoxes = planarMsg;
    featureMsg.poleBoxes = poleMsg;
    feature_obj_pub.publish(featureMsg);
    
    xy_clipped_global_pub_.publish(unavlib::cvt::cloud2msg(*xy_clipped_transformed, "odom"));
    non_ground_pub_.publish(unavlib::cvt::cloud2msg(*pc_nonground, "odom"));
    pole_global_pub_.publish(unavlib::cvt::cloud2msg(*pole_transformed, "odom"));
    plane_global_pub_.publish(unavlib::cvt::cloud2msg(*plane_transformed, "odom"));

    // pole_local visualization 
    pole_local_pub_.publish(unavlib::cvt::cloud2msg(*pc_pole, "odom"));

    ResetParam();

    vec_planar.clear();
    vec_pole.clear();
}

void feature::extract_plane_object(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Matrix4f pose_eigen,  Eigen::Matrix4f lidar_pose_eigen)
{    
    // 1. clip z height 
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_clipped_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI incloud_min, incloud_max;
    pcl::getMinMax3D(*in_cloud, incloud_min, incloud_max);
    clip_height(in_cloud, *pc_clipped_z, incloud_min.z + pl_z_axis_min_, incloud_max.z);

    // 2. voxelization(optional)
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    voxelize(pc_clipped_z, *pc_voxel, 0.2, 0.2, 1);

    // 3. region growing 
    std::vector<pcl::PointIndices> region_growing_clusters;
    region_growing(pc_voxel, region_growing_clusters);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_region (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_2d (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_2d_ransac (new pcl::PointCloud<pcl::PointXYZI>);

    // jslocalization::planars planar_objects;
    // planar_objects.header.frame_id = "odom";
    // planar_objects.header.stamp = ros::Time::now();
    // planar_objects.gps_odom = cvt::eigen2geoPose(pose_eigen);
    // planar_objects.odom = cvt::eigen2geoPose(lidar_pose_eigen);
    // planar_objects.index = planar_cnt;
    // planar_cnt++;

    planar_cnt = 0;
    for(int i=0; i<region_growing_clusters.size(); i++)
    {
        pcl::PointIndices::Ptr planar_inliers(new pcl::PointIndices());
        *planar_inliers = region_growing_clusters[i];
        pcl::copyPointCloud(*pc_voxel, *planar_inliers, *cloud_f);
        *planar_region += *cloud_f ;

        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_planar_2d(new pcl::PointCloud<pcl::PointXYZI>());
        voxelize(cloud_f, *voxel_planar_2d, 0.2, 0.2, 10000);
        for(int j=0; j<voxel_planar_2d->points.size(); j++)
        {
            voxel_planar_2d->points[j].z = incloud_min.z;
        }

        *planar_2d += *voxel_planar_2d;

        pcl::ModelCoefficients coefficients;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);

        coefficients = ransac_line(voxel_planar_2d, *inlierPoints , ransac_threads, ransac_threshold);
        *planar_2d_ransac += *inlierPoints;

        pcl::PointXYZI min_line, max_line;
        pcl::getMinMax3D(*inlierPoints, min_line, max_line);

        const auto pt_line_x = coefficients.values[0];
        const auto pt_line_y = coefficients.values[1];
        const auto pt_line_z = coefficients.values[2];
        const auto pt_direction_x = coefficients.values[3];
        const auto pt_direction_y = coefficients.values[4];
        const auto pt_direction_z = coefficients.values[5];

        float slope = pt_direction_y / pt_direction_x;
        float b = pt_line_y - slope*pt_line_x;
        float dist = abs(slope*min_line.x - min_line.y + b)/ sqrt(pow(slope,2)+1);
        if(dist > 1)
        {
            float temp  = min_line.y;    
            min_line.y = max_line.y;
            max_line.y = temp;
        }

        geometry_msgs::Point line_point_min, line_point_max;
        line_point_min.x = min_line.x;
        line_point_min.y = min_line.y;
        line_point_min.z = min_line.z;

        line_point_max.x = max_line.x;
        line_point_max.y = max_line.y;
        line_point_max.z = max_line.z;

        geometry_msgs::Point centroid;
        centroid.x = (line_point_min.x + line_point_max.x)/2;
        centroid.y = (line_point_min.y + line_point_max.y)/2;
        centroid.z = (line_point_min.z + line_point_max.z)/2;

        double length = sqrt(pow(line_point_max.x-line_point_min.x,2) + pow(line_point_max.y-line_point_min.y,2));
        // if(length<m_length_threshold)
        //     continue;

        jslocalization::planar planar_obj;
        planar_obj.header.frame_id = "odom";
        planar_obj.header.stamp = ros::Time::now();
        planar_obj.pc_planar = cvt::cloud2msg(*inlierPoints,"odom");
        planar_obj.index = planar_cnt;
        planar_obj.start = line_point_min;
        planar_obj.end = line_point_max;
        planar_obj.centroid = centroid;
        planar_obj.slope = slope;
        planar_obj.theta = atan2(pt_direction_y, pt_direction_x)*180/M_PI; // degree
        planar_obj.length = length;
        planar_cnt++;

        jslocalization::PlanarTrackBox planar_feature;
        planar_feature.header.frame_id = "odom";
        planar_feature.header.stamp = ros::Time::now();
        planar_feature.min_x = line_point_min.x - 0.3;
        planar_feature.max_x = line_point_max.x + 0.3;
        planar_feature.min_y = line_point_min.y - 0.3;
        planar_feature.max_y = line_point_max.y + 0.3;
        planar_feature.probability = 1;
        planar_feature.planar = planar_obj;
        vec_planar.push_back(planar_feature);

        std::cout<<"PL ["<<i<<"] x: "<<centroid.x<<" y: "<<centroid.y<<" theta: "<<planar_obj.theta<<" length: "<<length<<std::endl;
    }
    *pc_plane += *planar_region;

    pl_raw_pub_.publish(unavlib::cvt::cloud2msg(*in_cloud,"odom"));
    pl_z_clipped_pub_.publish(unavlib::cvt::cloud2msg(*pc_voxel,"odom"));
    pl_region_pub_.publish(unavlib::cvt::cloud2msg(*planar_region,"odom"));
    pl_2d_pub_.publish(unavlib::cvt::cloud2msg(*planar_2d, "odom"));
    pl_2d_ransac_pub_.publish(unavlib::cvt::cloud2msg(*planar_2d_ransac, "odom"));
}

void feature::extract_pole_object(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Matrix4f pose_eigen, Eigen::Matrix4f lidar_pose_eigen)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster4(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster5(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster6(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster1_tf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster2_tf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster3_tf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster4_tf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster5_tf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster6_tf(new pcl::PointCloud<pcl::PointXYZI>);

    // jslocalization::ObjTrackBoxes detected_poles;
    // detected_poles.header.frame_id = "odom";
    // detected_poles.header.stamp = ros::Time::now();
    // detected_poles.gps_odom =  cvt::eigen2geoPose(pose_eigen);
    // detected_poles.odom =  cvt::eigen2geoPose(lidar_pose_eigen);
    // detected_poles.index = pole_cnt;
    // pole_cnt++;

    // 1.Find bound
    pcl::PointXYZI incloud_min, incloud_max;
    pcl::getMinMax3D(*in_cloud, incloud_min, incloud_max);

    // 2. adaptive clustering 
    array8_t indices_array = create_array(in_cloud, po_z_axis_min_, po_z_axis_max_);

    float tolerance = 0.0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;

    for(int i = 0; i < region_max_; i++){
        tolerance += 0.1;
        if(indices_array[i].size() > po_cluster_size_min_) {
            boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(in_cloud, indices_array_ptr);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(tolerance);
            ec.setMinClusterSize(po_cluster_size_min_);
            ec.setMaxClusterSize(po_cluster_size_max_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(in_cloud);
            ec.setIndices(indices_array_ptr);
            ec.extract(cluster_indices);
          
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                    cluster->points.push_back(in_cloud->points[*pit]);
                }
                cluster->width = cluster->size();
                cluster->height = 1;
                cluster->is_dense = true;
                clusters.push_back(cluster);
            }
        }
    }

    // 3. height & radius filtering 
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pole_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i < clusters.size(); i++) 
    {
        *pc_cluster1 += *clusters[i];

        pcl::PointXYZI cluster_min, cluster_max;
        pcl::getMinMax3D(*clusters[i], cluster_min, cluster_max);

        if( abs(cluster_max.z - cluster_min.z) > po_height)
        {
            *pc_cluster2 += *clusters[i];

            unsigned int num_slice =  (int) (abs(cluster_max.z - cluster_min.z)/po_voxel_size) + 1 ;

            for(int j = 0; j < num_slice; j++)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pc_height_slice (new pcl::PointCloud<pcl::PointXYZI>);

                double clip_min = (double) (cluster_min.z + po_voxel_size*(j));
                double clip_max = (double) (cluster_min.z + po_voxel_size*(j+1));

                clip_height(clusters[i], *pc_height_slice, clip_min, clip_max);

                pcl::PointXYZI slice_min, slice_max;
                pcl::getMinMax3D(*pc_height_slice, slice_min, slice_max);

                if((slice_min.x-slice_max.x)*(slice_min.x-slice_max.x) + (slice_min.y-slice_max.y)*(slice_min.y-slice_max.y) < 4*po_radius*po_radius)
                {
                    *pc_pole_cluster += *pc_height_slice;
                    *pc_cluster3 += *pc_height_slice;     
                }
            } 
        }
    }

    pcl::transformPointCloud(*pc_cluster1, *pc_cluster1_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster2, *pc_cluster2_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster3, *pc_cluster3_tf, pose_eigen);
    test1_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster1, "odom"));
    test2_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster2, "odom"));
    test3_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster3, "odom"));

    // 4. euclidean clustering
    pole_cnt = 0;
    std::vector<pcl::PointIndices> cluster_indices_new;
    cluster_indices_new = euclidean_clustering(pc_pole_cluster, po_ec_tolerance, po_cluster_size_min_, po_cluster_size_max_);   

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_new.begin(); it != cluster_indices_new.end(); it++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_new(new pcl::PointCloud<pcl::PointXYZI>);
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster_new->points.push_back(pc_pole_cluster->points[*pit]);
        }
        cluster_new->width = cluster_new->size();
        cluster_new->height = 1;
        cluster_new->is_dense = true;

        *pc_cluster4 += *cluster_new;

        pcl::PointXYZI trunk_min, trunk_max;
        pcl::getMinMax3D(*cluster_new, trunk_min, trunk_max);
        if(trunk_max.z - trunk_min.z > po_trunk_height){

            *pc_cluster5 += *cluster_new;

            if(abs(trunk_min.z - incloud_min.z) < po_height_min)
            {
                *pc_pole += *cluster_new;
                *pc_cluster6 += *cluster_new;

                // for object tracker
                // jslocalization::ObjTrackBox pole_obj;
                // pole_obj.header.frame_id = "odom";
                // pole_obj.header.stamp = ros::Time::now();
                // pole_obj.min_x = trunk_min.x-0.3;
                // pole_obj.max_x = trunk_max.x+0.3;
                // pole_obj.min_y = trunk_min.y-0.3;
                // pole_obj.max_y = trunk_max.y+0.3;
                // pole_obj.probability = 1;
                // detected_poles.objlist.push_back(pole_obj);
                
                geometry_msgs::Point pole_centroid;
                pole_centroid.x = 0;
                pole_centroid.y = 0;
                pole_centroid.z = 0;

                jslocalization::pole pole_obj;
                pole_obj.header.frame_id = "odom";
                pole_obj.header.stamp = ros::Time::now();
                pole_obj.pc_pole = cvt::cloud2msg(*pc_pole,"odom");
                pole_obj.centroid = pole_centroid;
                pole_obj.radius = 0.0;
                pole_obj.covariance = 0.0;
                pole_obj.index = pole_cnt;
                pole_obj.height = trunk_max.z - trunk_min.z;

                pole_cnt++;

                jslocalization::PoleTrackBox pole_feature;
                pole_feature.header.frame_id = "odom";
                pole_feature.header.stamp = ros::Time::now();
                pole_feature.min_x = trunk_min.x-0.3;
                pole_feature.max_x = trunk_max.x+0.3;
                pole_feature.min_y = trunk_min.y-0.3;
                pole_feature.max_y = trunk_max.y+0.3;
                pole_feature.probability = 1;
                pole_feature.pole = pole_obj;
                vec_pole.push_back(pole_feature);
            }
        }
    }

    // pole_obj_pub.publish(detected_poles);

    pcl::transformPointCloud(*pc_cluster4, *pc_cluster4_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster5, *pc_cluster5_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster6, *pc_cluster6_tf, pose_eigen);
    test4_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster4, "odom"));
    test5_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster5, "odom"));
    test6_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster6, "odom"));
}

std::vector<pcl::PointIndices> feature::euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, float tolerance, int min_size, int max_size)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (in_cloud);  //KdTree 생성 
    std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud (in_cloud);       // 입력   
    ec.setClusterTolerance (tolerance);  // 2cm  
    ec.setMinClusterSize (min_size);     // 최소 포인트 수 
    ec.setMaxClusterSize (max_size);   // 최대 포인트 수
    ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
    ec.extract (cluster_indices);   // 군집화 적용 

    return cluster_indices;
}

array8_t feature::create_array(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, double z_axis_min, double z_axis_max)
{
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(in_cloud);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(z_axis_min, z_axis_max);
    pt.filter(*pc_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr z_clipped(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_cloud, *pc_indices, *z_clipped);

    array8_t indices_array;
    for(int i = 0; i <  pc_indices->size(); i++){
        float range = 0.0;
        for(int j = 0; j < region_max_; j++){
            float d2 = in_cloud->points[(*pc_indices)[i]].x * in_cloud->points[(*pc_indices)[i]].x +
                     in_cloud->points[(*pc_indices)[i]].y * in_cloud->points[(*pc_indices)[i]].y +
                     in_cloud->points[(*pc_indices)[i]].z * in_cloud->points[(*pc_indices)[i]].z;
            if(d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) {
                indices_array[j].push_back((*pc_indices)[i]);
                break;
            }
            range += regions_[j];
        }
    }

    return indices_array;
}

pcl::ModelCoefficients feature::ransac_line(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, int threads, float threshold)
{
    // ransac to find line coefficients
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr rs_inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
    seg.setInputCloud (in_cloud);                 //입력 
    seg.setModelType (pcl::SACMODEL_LINE);    //적용 모델  // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
    seg.setMaxIterations (threads);               //최대 실행 수
    seg.setDistanceThreshold (threshold);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
    seg.segment (*rs_inliers, *coefficients);    //세그멘테이션 적용 
    pcl::copyPointCloud<pcl::PointXYZI>(*in_cloud, *rs_inliers, *inlierPoints);
    
    out_cloud = *inlierPoints;
    return *coefficients;
}

void feature::voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double leaf_x, double leaf_y, double leaf_z)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    voxel_filter.setInputCloud(in_cloud);
    voxel_filter.setLeafSize(leaf_x,leaf_y,leaf_z);
    voxel_filter.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::remove_side_x(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_remove;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_remove.setInputCloud(in_cloud);
    pass_remove.setFilterFieldName("x");
    pass_remove.setFilterLimits(-max_range, max_range);

    pass_remove.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::remove_side_y(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_remove;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_remove.setInputCloud(in_cloud);
    pass_remove.setFilterFieldName("y");
    pass_remove.setFilterLimits(-max_range, max_range);

    pass_remove.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::clip_height(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double min_height, double max_height)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_height;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_height.setInputCloud(in_cloud);
    pass_height.setFilterFieldName("z");
    pass_height.setFilterLimits(min_height, max_height);

    pass_height.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::region_growing(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, std::vector<pcl::PointIndices> &out_cluster)
{
    std::cout<<"\033[1;36m[RegionGrowing]\033[0m Region growing is started "<< std::endl;
    std::clock_t start = std::clock();

    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (in_cloud);
    normal_estimator.setKSearch (rg_num_ksearch);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (rg_mincluster);
    reg.setMaxClusterSize (100000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (rg_numneighbor);
    reg.setInputCloud (in_cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (rg_smoothness / 180.0 * M_PI); //3.0
    reg.setCurvatureThreshold (rg_curvature); // 1.0 

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    out_cluster = clusters;

    std::clock_t end = std::clock();
    std::cout<<"\033[1;36m[RegionGrowing]\033[0m Running Time : " << static_cast<double>((end-start))/CLOCKS_PER_SEC << std::endl;

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

}

// void PoleObject::RoadPlaneExtration(const sensor_msgs::PointCloud2::ConstPtr &lidar)
void feature::road_plane_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud)
{    
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn_org;

    laserCloudIn = *in_cloud;
    laserCloudIn_org = *in_cloud;

    // 2. Sort on Z-axis value.
    sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp); 

    // 3.Error point removal
    // As there are some error mirror reflection under the ground, 
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<pcl::PointXYZI>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -removal_height_*sensor_height_){
            it++;
        }else{
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);

    extract_initial_seeds(laserCloudIn, *g_seeds_pc);
    g_ground_pc = g_seeds_pc;

    for(int i=0;i<num_iter_;i++){
        estimate_plane(g_seeds_pc);
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }else{
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }
    g_ground_pc->width = 1;
    g_ground_pc->height = g_ground_pc->points.size();

    g_not_ground_pc->width = 1;
    g_not_ground_pc->height = g_not_ground_pc->points.size();

    for(int i=0; i<g_ground_pc->points.size(); i++)
    {
        if(g_ground_pc->points[i].intensity > min_intensity && g_ground_pc->points[i].intensity < max_intensity )
            pc_marking->points.push_back(g_ground_pc->points[i]);
    }

    out_cloud = *g_not_ground_pc;
}


void feature::estimate_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud){
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*in_cloud, cov, pc_mean);
    // Singular Value Decomposition: SVD
    // Used in JacobiSVD to indicate that the square matrix U is to be computed. 
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU); 
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // std::cout << "normal value : " << normal_ << std::endl;
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
 
    // return the equation parameters
}

void feature::extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &out_cloud){
    // LPR is the mean of low point representative
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<num_lpr_;i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    filtered_cloud->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + th_seeds_){
            filtered_cloud->points.push_back(p_sorted.points[i]);
        }
    }
    out_cloud = *filtered_cloud;
    // return seeds points
}