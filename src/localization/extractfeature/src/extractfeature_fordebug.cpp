#include "extractfeature.h"

int regions_[100];

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z<b.z;
}

using namespace unavlib;
feature::feature()
{
    static ros::Subscriber nodegen_sub = m_nh.subscribe<jslocalization::node>("/jslocalization/feature/nodeIn", 10, &feature::pointCloudCallback, this);

    xy_clipped_pub_ =  m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/xy_clipped", 10);

    ground_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/ground", 1);
    non_ground_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/non_ground", 1);
    marking_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/road_marking", 1);

    po_z_clipped_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/z_clipped", 10);
    po_cluster_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/clusters", 10);
    pole_frag_pub_ =  m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/candidates", 10);
    pole_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/pole", 10);
    pole_test_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/pole_test", 10);

    test1_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test1", 10);
    test2_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test2", 10);
    test3_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test3", 10);
    test4_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test4", 10);
    test5_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test5", 10);
    test6_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/test6", 10);

    pl_z_clipped_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/z_clipped", 10);
    pl_cluster_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/clusters", 10);
    plane_candi_pub_ =  m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/candidates", 10);
    plane_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/planar", 10);
    plane2D_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/planar2D", 10);    
    plane2D_rs_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/planar2D_ransac", 10);    
    plane_removed_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/no_plane", 10);    

    pole_trans_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/pole/pole_transformed", 10);
    plane_trans_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/plane/plane_transformed", 10);

    jslocalization_feature_pub_ = m_nh.advertise<jslocalization::feature>("/jslocalization_feature", 10);

    test_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/feature/test", 10);

    AllocateMemory();
    SetParam();
}

void feature::AllocateMemory()
{
    pc_pole.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_plane.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_marking.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_nonground.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pc_pole_test.reset(new pcl::PointCloud<pcl::PointXYZI>());

}

void feature::ResetParam()
{
    pc_pole->clear();
    pc_plane->clear();
    pc_marking->clear();
    pc_nonground->clear();
    pole_vec.clear();
    pc_pole_test->clear();
}

void feature::SetParam()
{   
    std::vector<double> lidar2body;
    if(m_nh.getParam("/NodeGen/lidar2body", lidar2body)) {
        if(lidar2body.size()==7) {
            geometry_msgs::Pose geo_pose;

            geo_pose.position.x = lidar2body[0];
            geo_pose.position.y = lidar2body[1];
            geo_pose.position.z = lidar2body[2];
            geo_pose.orientation.x = lidar2body[3];
            geo_pose.orientation.y = lidar2body[4];
            geo_pose.orientation.z = lidar2body[5];
            geo_pose.orientation.w = lidar2body[6];

            tf_lidar2body = unavlib::cvt::geoPose2eigen(geo_pose);
        }else {
            ROS_ERROR("TF MUST BE X,Y,Z,Qx,Qy,Qw,Qz");
            exit(0);
        }
    }

    m_nh.param<std::string>("/System/model", sensor_model_, "os1-64"); // VLP-16, HDL-32E, HDL-64E

    m_nh.param("/ground/sensor_height", sensor_height_, 2.0);
    m_nh.param("/ground/removal_height", removal_height_, 1.5);
    m_nh.param("/ground/num_seg", num_seg_, 1);
    m_nh.param("/ground/num_iter", num_iter_, 3);
    m_nh.param("/ground/num_lpr", num_lpr_, 20);
    m_nh.param("/ground/th_seeds", th_seeds_, 1.2);
    m_nh.param("/ground/th_dist", th_dist_, 0.3);

    m_nh.param("/road/max_intensity", max_intensity, 1000.0);
    m_nh.param("/road/min_intensity", min_intensity, 500.0);
    
    m_nh.param<float>("/pole/x_clipped", po_x_width, 50.0);
    m_nh.param<float>("/pole/y_clipped", po_y_width, 50.0);
    m_nh.param<float>("/pole/z_axis_min", po_z_axis_min_, -1.0);
    m_nh.param<float>("/pole/z_axis_max", po_z_axis_max_, 10.0);
    m_nh.param<int>("/pole/cluster_size_min", po_cluster_size_min_, 5);
    m_nh.param<int>("/pole/cluster_size_max", po_cluster_size_max_, 700000);
    m_nh.param<float>("/pole/height_pole", po_height, 2.5);
    m_nh.param<float>("/pole/height_max", po_height_max, 2.5);
    m_nh.param<float>("/pole/height_min", po_height_min, 1.0);
    m_nh.param<float>("/pole/voxel_size", po_voxel_size, 0.2);
    m_nh.param<float>("/pole/pole_radius", po_radius, 3.0);
    m_nh.param<float>("/pole/ec_tolerance", po_ec_tolerance, 0.2);
    m_nh.param<float>("/pole/height_trunk", po_trunk_height, 0.5);

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

    if (sensor_model_ == "os1-64") { // if ec tolerance is 0.1
        regions_[0] = 11; regions_[1] = 11; regions_[2] = 11; regions_[3] = 11; regions_[4] = 11;
        regions_[5] = 11; regions_[6] = 11; regions_[7] = 11; 
    }

    // if (sensor_model_ == "os1-64") { // if ec tolerance is 0.2
    //     regions_[0] = 22; regions_[1] = 22; regions_[2] = 22; regions_[3] = 22; regions_[4] = 22;
    //     regions_[5] = 22; regions_[6] = 22; regions_[7] = 22; 
    // }

    // if (sensor_model_ == "os1-64") { // if ec tolerance is 0.3
    //     regions_[0] = 33; regions_[1] = 33; regions_[2] = 33; regions_[3] = 33; regions_[4] = 33;
    //     regions_[5] = 33; regions_[6] = 33; regions_[7] = 33; 
    // }

    std::cout<<"\033[1;37m[SetParam]\033[0m num_lpr: "<<num_lpr_<<" th_seeds: "<<th_seeds_<<" th_dist: "<<th_dist_<<std::endl;
}

void feature::pointCloudCallback(const jslocalization::node::ConstPtr &node)
{
    Eigen::Matrix4f gps_eigen = cvt::geoPose2eigen(node->gps_odom);

    sensor_msgs::PointCloud2::Ptr cloudmsg_in (new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_raw(new pcl::PointCloud<pcl::PointXYZI>);

    *cloudmsg_in = node->lidar;
    pcl::fromROSMsg(*cloudmsg_in, *pcl_pc_raw);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_clipped_x(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_transformed (new pcl::PointCloud<pcl::PointXYZI>);

    RemoveSideX(pcl_pc_raw, *pcl_pc_clipped_x, po_x_width); ///50
    RemoveSideY(pcl_pc_clipped_x, *pcl_pc_in, po_y_width); //50

    pcl::transformPointCloud(*pcl_pc_in, *pcl_pc_transformed, gps_eigen);
    xy_clipped_pub_.publish(unavlib::cvt::cloud2msg(*pcl_pc_transformed, "odom"));

#if 0
    static int cnt = 0;
    std::string path = "/media/jsg/T7/pcd/scan/raw/";
    path = path + std::to_string(cnt) + ".pcd";
    pcl::io::savePCDFile<pcl::PointXYZI> (path, *pcl_pc_transformed);
    cnt++;
#endif 

    RoadPlaneExtration(pcl_pc_in);  //pcl_pc_in

    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_removed(new pcl::PointCloud<pcl::PointXYZI>);

    std::clock_t start = std::clock();
    // ExtractPlaneObject(pcl_pc_in,*plane_removed); // pcl_pc_in
    ExtractPlaneObject(pc_nonground,*plane_removed); // pcl_pc_in

    static int test = 0;
    // if(test==1)
        ExtractPoleObject(pc_nonground, gps_eigen);
    // test++;
    std::clock_t end = std::clock();
    // std::cout << "Running Time of Feature Extraction: " << static_cast<double>((end-start))/CLOCKS_PER_SEC << std::endl;

    plane_removed_pub_.publish(unavlib::cvt::cloud2msg(*plane_removed, "odom"));

    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr marking_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_test_transformed (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*pc_plane, *plane_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    pcl::transformPointCloud(*pc_pole, *pole_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    pcl::transformPointCloud(*pc_pole_test, *pole_test_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));

    pole_trans_pub_.publish(unavlib::cvt::cloud2msg(*pole_transformed, "odom"));
    plane_trans_pub_.publish(unavlib::cvt::cloud2msg(*plane_transformed, "odom"));
    pole_test_pub_.publish(unavlib::cvt::cloud2msg(*pole_test_transformed, "odom"));

    // pcl::transformPointCloud(*pc_marking, *marking_transformed2, tf_lidar2body);
    // pcl::transformPointCloud(*marking_transformed2, *marking_transformed, unavlib::cvt::geoPose2eigen(nodegen->gps_pose));

    pcl::transformPointCloud(*pc_nonground, *nonground_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    non_ground_pub.publish(unavlib::cvt::cloud2msg(*nonground_transformed, "odom"));


    jslocalization::feature jslocalization_feature;

    jslocalization_feature.header.frame_id = "odom";
    jslocalization_feature.header.stamp = node->header.stamp;
    jslocalization_feature.index = node->index;
    jslocalization_feature.gps_odom = node->gps_odom;
    jslocalization_feature.odom = node->odom;

    pcl::PointCloud<pcl::PointXYZ> pc_pole_obs;

    for(int i =0; i<pole_vec.size(); i++)
    {
        jslocalization_feature.pole_obs.push_back(pole_vec[i]);
        pcl::PointXYZ temp;
        temp.x = pole_vec[i].x;
        temp.y = pole_vec[i].y;
        temp.z = 0;
        pc_pole_obs.points.push_back(temp);
    }

    jslocalization_feature.pc_pole_obs = unavlib::cvt::cloud2msg(pc_pole_obs, "odom");
    jslocalization_feature_pub_.publish(jslocalization_feature);


    pcl::PointCloud<pcl::PointXYZ>::Ptr test_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(pc_pole_obs, *test_transformed, unavlib::cvt::geoPose2eigen(node->gps_odom));
    test_pub.publish(unavlib::cvt::cloud2msg(*test_transformed, "odom"));

    // jslocalization_feature
    // jsslam_feature.lidar_global = nodegen->lidar_global;
    // jsslam_feature.lidar_local = nodegen->lidar_local;
    // jsslam_feature.pole_object = unavlib::cvt::cloud2msg(*pole_transformed, "odom");
    // jsslam_feature.plane_object = unavlib::cvt::cloud2msg(*plane_transformed, "odom");
    // jsslam_feature.marking_object = unavlib::cvt::cloud2msg(*marking_transformed, "odom");
    

    // std::cout << "Feature Index: " << nodegen->node_index << std::endl;

    ResetParam();
}

void feature::pc_transform(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, Eigen::Matrix4f pose_eigen)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_transform (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*in_cloud, *pc_transform, pose_eigen);
    out_cloud = *pc_transform;
}

void feature::ExtractPlaneObject(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud)
{
    // variables
    cluster_t clusters;
    std::vector<pcl::PointIndices> clusters_indices;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cluster_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar2D_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar2D_ransac_ptr (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> planar_removed_extract;
    pcl::PointIndices::Ptr planar_inliers(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_removed_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    
    // 1. adaptive clustering 
    array8_t indices_array = create_array(in_cloud, pl_z_axis_min_, pl_z_axis_max_);
    apaptive_clustering(in_cloud, clusters, clusters_indices, indices_array, pl_cluster_size_min_, pl_cluster_size_max_);

    // std::cout << "clusters size : " << clusters.size() << std::endl;
    if(clusters.size()>0)
    {
        for(int i = 0; i < clusters.size(); i++) 
        {
            *pc_cluster += *clusters[i];

            pcl::PointXYZI cluster_min, cluster_max;
            pcl::getMinMax3D(*clusters[i], cluster_min, cluster_max);

             extract_inliers(in_cloud, clusters_indices[i], *planar_removed_ptr);


            if( abs(cluster_max.z - cluster_min.z) > pl_height)//&& abs(cluster_min.z - pl_z_axis_min_) < pl_height_min)
            {   
                *pc_cluster_filtered += *clusters[i];

                // extract_inliers(in_cloud, clusters_indices[i], *planar_removed_ptr);

                // 2. voxelization
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxel(new pcl::PointCloud<pcl::PointXYZI>);
                voxelize(clusters[i], *ptr_voxel, 0.2, 0.2, 0.2);

                // 3. region growing
                std::vector<pcl::PointIndices> rg_clusters;
                region_growing(ptr_voxel, rg_clusters);

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

                for(int i=0; i<rg_clusters.size(); i++)
                {
                    // std::cout <<i<<"th cluster size : " <<rg_clusters[i].indices.size()<<std::endl;
                    pcl::PointIndices::Ptr planar_inliers(new pcl::PointIndices());
                    *planar_inliers = rg_clusters[i];
                    pcl::copyPointCloud(*ptr_voxel, *planar_inliers, *cloud_f);
                    *planar_ptr += *cloud_f ;

                    // 4. 2D projection
                    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized2D(new pcl::PointCloud<pcl::PointXYZI>());
                    voxelize(cloud_f, *voxelized2D, 0.2, 0.2, 10000);
                    for(int j=0; j<voxelized2D->points.size(); j++)
                    {
                        voxelized2D->points[j].z = pl_z_axis_min_;
                    }
                    *planar2D_ptr += *voxelized2D;

                    // 5. ransac to find 2D line 
                    pcl::ModelCoefficients coefficients;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);

                    coefficients = ransac_line(voxelized2D, *inlierPoints , ransac_threads, ransac_threshold);
                    *planar2D_ransac_ptr += *inlierPoints;
                }

            }

        }
        out_cloud = *planar_removed_ptr;
    }
    else{
        out_cloud = *in_cloud;
    }
    
    *pc_plane = *planar_ptr;

    pl_cluster_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster, "odom"));
    plane_candi_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster_filtered, "odom"));
    plane_pub_.publish(unavlib::cvt::cloud2msg(*planar_ptr, "odom"));
    plane2D_pub_.publish(unavlib::cvt::cloud2msg(*planar2D_ptr, "odom"));
    plane2D_rs_pub_.publish(unavlib::cvt::cloud2msg(*planar2D_ransac_ptr, "odom"));
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

void feature::apaptive_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, cluster_t &out_clusters, std::vector<pcl::PointIndices> &out_indices, array8_t indices_array, int cluster_min, int cludster_max)
{
    float tolerance = 0.0;

    cluster_t clusters;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<pcl::PointIndices> clusters_indices;

    for(int i = 0; i < region_max_; i++) {
        tolerance += 0.1;
        if(indices_array[i].size() > cluster_min) {
            boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(in_cloud, indices_array_ptr);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(tolerance);
            ec.setMinClusterSize(cluster_min);
            ec.setMaxClusterSize(cludster_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(in_cloud);
            ec.setIndices(indices_array_ptr);
            ec.extract(cluster_indices);

      
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                    cluster->points.push_back(in_cloud->points[*pit]);
                    inliers->indices.push_back(*pit);
                }
                cluster->width = cluster->size();
                cluster->height = 1;
                cluster->is_dense = true;
                clusters.push_back(cluster);
                clusters_indices.push_back(*inliers);
            }        
        }
    }
    out_clusters = clusters;
    out_indices = clusters_indices;
}

void feature::extract_inliers(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZI> &out_cloud)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

    *inliers = indices;

    extract.setInputCloud(in_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    in_cloud.swap(cloud_f);

    out_cloud = *in_cloud;
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

void feature::ExtractPoleObject(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Matrix4f pose_eigen)
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

#if 0
    static int cnt =0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    *temp = *in_cloud;
    temp->width = 1;
    temp->height = temp->size();
    std::string path = "/media/jsg/T7/pcd/scan_raw/";
    path = path + std::to_string(cnt) + ".pcd";
    pcl::io::savePCDFile<pcl::PointXYZI> (path, *temp);

    #endif 

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pole_cluster(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointXYZI cloud_min, cloud_max;
    pcl::getMinMax3D(*in_cloud, cloud_min, cloud_max);

    for(int i = 0; i < clusters.size(); i++) {
        *pc_cluster1 += *clusters[i];
#if 0
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZI>);
        *temp1 = *clusters[i];
        temp1->width = 1;
        temp1->height = temp1->size();
        static int cnt1=0;
        std::string path1 = "/media/jsg/T7/pcd/scan1/cluster1/";
        path1 = path1 + std::to_string(cnt1) + ".pcd";
        pcl::io::savePCDFile<pcl::PointXYZI> (path1, *temp1);
        cnt1++;   
        # endif

        pcl::PointXYZI cluster_min, cluster_max;
        pcl::getMinMax3D(*clusters[i], cluster_min, cluster_max);

        if( abs(cluster_max.z - cluster_min.z) > po_height)// && abs(cluster_min.z - cloud_min.z) < po_height_min)
        {
            *pc_cluster2 += *clusters[i];
#if 0
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp3(new pcl::PointCloud<pcl::PointXYZI>);
            *temp3 = *clusters[i];
            temp3->width = 1;
            temp3->height = temp3->size();
            static int cnt3=0;
            std::string path3 = "/media/jsg/T7/pcd/scan1/cluster3/";
            path3 = path3 + std::to_string(cnt3) + ".pcd";
            pcl::io::savePCDFile<pcl::PointXYZI> (path3, *temp3);
            cnt3++;   
#endif

            unsigned int num_slice =  (int) (abs(cluster_max.z - cluster_min.z)/po_voxel_size) + 1 ;

            for(int j = 0; j < num_slice; j++)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pc_height_slice (new pcl::PointCloud<pcl::PointXYZI>);

                double clip_min = (double) (cluster_min.z + po_voxel_size*(j));
                double clip_max = (double) (cluster_min.z + po_voxel_size*(j+1));

                ClipHeight(clusters[i], *pc_height_slice, clip_min, clip_max);

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
#if 0
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZI>);
    *temp2 = *pc_pole_cluster;
    temp2->width = 1;
    temp2->height = temp2->size();
    static int cnt2=0;
    std::string path2 = "/media/jsg/T7/pcd/scan1/cluster2/";
    path2 = path2 + std::to_string(cnt2) + ".pcd";
    if(temp2->size()!=0){
        pcl::io::savePCDFile<pcl::PointXYZI> (path2, *temp2);
        cnt2++; 
    }
#endif           
    
    pcl::transformPointCloud(*pc_cluster1, *pc_cluster1_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster2, *pc_cluster2_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster3, *pc_cluster3_tf, pose_eigen);

    test1_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster1_tf, "odom"));
    test2_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster2_tf, "odom"));
    test3_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster3_tf, "odom"));

    // pole_frag_pub_.publish(unavlib::cvt::cloud2msg(*pc_pole_cluster, "odom"));

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
        // clusters.push_back(cluster);
#if 0
        static int cnt3=0;
        std::string path3 = "/media/jsg/T7/pcd/scan1/cluster3/";
        path3 = path3 + std::to_string(cnt3) + ".pcd";
        pcl::io::savePCDFile<pcl::PointXYZI> (path3, *cluster_new);
        cnt3++; 
        #endif

        *pc_cluster4 += *cluster_new;

        pcl::PointXYZI trunk_min, trunk_max;
        pcl::getMinMax3D(*cluster_new, trunk_min, trunk_max);
        if(trunk_max.z - trunk_min.z > po_trunk_height){
            *pc_pole_test += *cluster_new;

            *pc_cluster5 += *cluster_new;

#if 0
            static int cnt4=0;
            std::string path4 = "/media/jsg/T7/pcd/scan1/cluster4/";
            path4 = path4 + std::to_string(cnt4) + ".pcd";
            pcl::io::savePCDFile<pcl::PointXYZI> (path4, *cluster_new);
            cnt4++; 
            #endif


            if(abs(trunk_min.z - cloud_min.z) < po_height_min){
                *pc_pole += *cluster_new;
                *pc_cluster6 += *cluster_new;


                Eigen::Vector4f pole_centroid;
                pcl::compute3DCentroid (*cluster_new, pole_centroid);
                geometry_msgs::Point temp;
                temp.x = pole_centroid(0);
                temp.y = pole_centroid(1);
                // std::cout << "[feature extraction] x: " << temp.x << " y: " << temp.y << " dist : " << sqrt(pow(temp.x,2)+ pow(temp.y, 2)) << std::endl;
                pole_vec.push_back(temp);

                // static int cnt_cluster=0;
                // std::string path = "/media/jsg/T7/pcd/scan/pole_r/";
                // path = path + std::to_string(cnt_cluster) + ".pcd";
                // pcl::io::savePCDFile<pcl::PointXYZI> (path, *cluster_new);
                // cnt_cluster++;   
#if 0
                static int cnt5=0;
                std::string path5 = "/media/jsg/T7/pcd/scan_cluster/";
                path5 = path5 + std::to_string(cnt5) + ".pcd";
                pcl::io::savePCDFile<pcl::PointXYZI> (path5, *cluster_new);
                cnt5++; 
#endif  

            }
        }
    }


    // std::string path6 = "/media/jsg/T7/pcd/scan_cluster/";
    // path6 = path6 + std::to_string(cnt) + ".pcd";
    // pcl::io::savePCDFile<pcl::PointXYZI> (path6, *pc_cluster6);
    // cnt++; 


    pcl::transformPointCloud(*pc_cluster4, *pc_cluster4_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster5, *pc_cluster5_tf, pose_eigen);
    pcl::transformPointCloud(*pc_cluster6, *pc_cluster6_tf, pose_eigen);

    test4_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster4_tf, "odom"));
    test5_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster5_tf, "odom"));
    test6_pub_.publish(unavlib::cvt::cloud2msg(*pc_cluster6_tf, "odom"));

    // pole_test_pub_.publish(unavlib::cvt::cloud2msg(*pole_candidates, "odom"));
    pole_pub_.publish(unavlib::cvt::cloud2msg(*pc_pole, "odom"));
}

void feature::RemoveSideX(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_remove;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_remove.setInputCloud(in_cloud);
    pass_remove.setFilterFieldName("x");
    pass_remove.setFilterLimits(-max_range, max_range);

    pass_remove.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::RemoveSideY(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double max_range)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_remove;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_remove.setInputCloud(in_cloud);
    pass_remove.setFilterFieldName("y");
    pass_remove.setFilterLimits(-max_range, max_range);

    pass_remove.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::ClipHeight(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI> &out_cloud, double min_height, double max_height)
{
    static pcl::PassThrough<pcl::PointXYZI> pass_height;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pass_height.setInputCloud(in_cloud);
    pass_height.setFilterFieldName("z");
    pass_height.setFilterLimits(min_height, max_height);

    pass_height.filter(*filtered_cloud);

    out_cloud = *filtered_cloud;
}

void feature::RansacNormalPlanar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>& dst)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZI>);

    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setSearchMethod (kdtree);
    ne.setInputCloud (cloud_in);
    ne.setKSearch (num_normal_kd); // num_normal_kd 100
    ne.compute (*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (normal_weight); //0.5
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloud_in);
    seg.setInputNormals (cloud_normals);

    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI>);
    extract.filter (*cloud_plane);
    // ROS_INFO ("PointCloud representing the planar component: %zu data points.", cloud_plane->points.size ());

    dst = *cloud_plane;

}

void feature::region_growing(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, std::vector<pcl::PointIndices> &out_cluster)
{
    // std::cout<<"\033[1;36m[RegionGrowing]\033[0m Region growing is started "<< std::endl;
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
    // std::cout<<"\033[1;36m[RegionGrowing]\033[0m Running Time : " << static_cast<double>((end-start))/CLOCKS_PER_SEC << std::endl;

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

}

// void PoleObject::RoadPlaneExtration(const sensor_msgs::PointCloud2::ConstPtr &lidar)
void feature::RoadPlaneExtration(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud)
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

    ExtractInitialSeeds(laserCloudIn, *g_seeds_pc);
    g_ground_pc = g_seeds_pc;

    for(int i=0;i<num_iter_;i++){
        EstimatePlane(g_seeds_pc);
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

    *pc_nonground = *g_not_ground_pc;

    // ground_pub.publish(unavlib::cvt::cloud2msg(*g_ground_pc, "odom"));
    // non_ground_pub.publish(unavlib::cvt::cloud2msg(*pc_nonground, "odom"));
    // marking_pub.publish(unavlib::cvt::cloud2msg(*pc_marking, "odom"));

#if 0
    static int cnt_g=0;
    std::string path_ng = "/media/jsg/T7/pcd/scan/non_ground/";
    path_ng = path_ng + std::to_string(cnt_g) + ".pcd";
    pcl::io::savePCDFile<pcl::PointXYZI> (path_ng, *g_not_ground_pc);

    std::string path_g = "/media/jsg/T7/pcd/scan/ground/";
    path_g = path_g + std::to_string(cnt_g) + ".pcd";
    pcl::io::savePCDFile<pcl::PointXYZI> (path_g, *g_ground_pc);
    cnt_g++;
#endif


}


void feature::EstimatePlane(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud){
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

void feature::ExtractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &out_cloud){
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