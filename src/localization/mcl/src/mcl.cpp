#include "mcl.h"

using namespace unavlib;

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z < b.z;
}
bool planar_cmp(pcl::PointXYZ a, pcl::PointXYZ b){
    return a.z > b.z;
}
bool particle_cmp(particle a, particle b){
    return a.score > b.score;
}

mcl::mcl()
{   
    SetParam();
    // static ros::Subscriber feature_sub = m_nh.subscribe<jslocalization::feature>("/jslocalization_feature", 10, &mcl::feature_callback, this);
    static ros::Subscriber hdmap_pole_sub = m_nh.subscribe<jslocalization::poles>("/jslocalization/nodegen/pole_info", 100, &mcl::hdmap_pole_callback, this);
    static ros::Subscriber hdmap_planar_sub = m_nh.subscribe<jslocalization::planars>("/jslocalization/nodegen/planar_info", 10, &mcl::hdmap_planar_callback, this);
    static ros::Timer timer = m_nh.createTimer(ros::Duration(0.1), &mcl::timer_callback,this);
    // static ros::Subscriber pole_tracked_sub = m_nh.subscribe<jslocalization::PoleTrackBoxes>("/jslocalization/sort/pole_tracked_boxes", 10, &mcl::pole_callback, this);
    // static ros::Subscriber planar_tracked_sub = m_nh.subscribe<jslocalization::PlanarTrackBoxes>("/jslocalization/sort/planar_tracked_boxes", 10, &mcl::planar_callback, this);
    static ros::Subscriber pole_tracked_sub = m_nh.subscribe<jslocalization::PoleTrackBoxes>("/jslocalization/feature/pole_trackboxes", 10, &mcl::pole_callback, this);
    static ros::Subscriber planar_tracked_sub = m_nh.subscribe<jslocalization::PlanarTrackBoxes>("/jslocalization/feature/planar_trackboxes", 10, &mcl::planar_callback, this);
    static ros::Subscriber feature_sub = m_nh.subscribe<jslocalization::FeatureBoxes>("/jslocalization/feature/feautre_boxes", 10, &mcl::feature_callback, this);

    m_pub_mclPose = m_nh.advertise<nav_msgs::Odometry>("jslocalization/mcl/pose",1);
    m_pub_mclParticles = m_nh.advertise<sensor_msgs::PointCloud2>("jslocalization/mcl/particles",1);
    m_pub_mclLiDAR = m_nh.advertise<sensor_msgs::PointCloud2>("jslocalization/mcl/lidar",1);
    m_pub_mclInfo = m_nh.advertise<jslocalization::mcl>("jslocalization/mcl/info",1);
    m_pub_markers = m_nh.advertise<visualization_msgs::MarkerArray>("jslocalization/mcl/markers",1);

    result_pc_pub = m_nh.advertise<sensor_msgs::PointCloud2>("jslocalization/sort/pole_taracked_result",1);
    m_pub_test = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/test",1);

    m_pub_scan_planar = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/planar_scan",1);
    m_pub_map_planar = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/map_scan_planar",1);
    m_pub_map_pole = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/map_scan_pole",1);


    m_pub_pole = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/pole_cloud",1);
    m_pub_planar = m_nh.advertise<sensor_msgs::PointCloud2>("/jslocalization/mcl/planar_cloud",1);

    m_bestpose = Eigen::Matrix4f::Identity();

    pole_map_received = false;
    planar_map_received = false;
    first_run = true;

    map_pole_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    selected_poles.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_planar_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    scan_planar_prt.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void mcl::SetParam()
{    
    m_nh.param("/mcl/particle_min",m_param_particleMin,1000);
    m_nh.param("/mcl/particle_max",m_param_particleMax,10000);
    m_nh.param("/mcl/pole_max",m_param_pole_max,3);
    m_nh.param("/mcl/planar_max",m_param_planar_max,1);
    m_nh.param<float>("/mcl/static_velocity",m_static_velocity,1.5);
    m_nh.param<float>("/mcl/planar_cov",m_planar_cov,1.5);
    m_nh.param<float>("/mcl/theta_noise",m_theta_noise, 1.0);


    std::vector<double> covOdom;
    m_param_cov = Eigen::VectorXf(6);
    if(m_nh.getParam("/mcl/covariance_odom",covOdom))
    {
        if(covOdom.size() == 6)
        {
            m_param_cov[0] = covOdom[0];
            m_param_cov[1] = covOdom[1];
            m_param_cov[2] = covOdom[2];
            m_param_cov[3] = covOdom[3];
            m_param_cov[4] = covOdom[4];
            m_param_cov[5] = covOdom[5];
        }
    }

}

void mcl::timer_callback(const ros::TimerEvent& event)
{
    jslocalization::mcl mclInfo;
    mclInfo.lidar = cvt::cloud2msg(*selected_poles);
    mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
    m_pub_mclInfo.publish(mclInfo);
}

void mcl::feature_callback(const jslocalization::FeatureBoxes::ConstPtr &msg) 
{
    vec_tracked_pole.push_back(msg->poleBoxes);
    vec_tracked_planar.push_back(msg->planarBoxes);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_pcAll(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<vec_tracked_pole[0].objlist.size(); i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr pole_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(vec_tracked_pole[0].objlist[i].pole.pc_pole, *pole_pc);
        *pole_pcAll += *pole_pc;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*pole_pcAll, *pole_transformed, unavlib::cvt::geoPose2eigen(msg->gps_odom));
    m_pub_pole.publish(unavlib::cvt::cloud2msg(*pole_transformed, "odom"));

    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_pcAll(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<vec_tracked_planar[0].objlist.size(); i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr planar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(vec_tracked_planar[0].objlist[i].planar.pc_planar, *planar_pc);
        *planar_pcAll += *planar_pc;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*planar_pcAll, *planar_transformed, unavlib::cvt::geoPose2eigen(msg->gps_odom));
    m_pub_planar.publish(unavlib::cvt::cloud2msg(*planar_transformed, "odom"));


    if(vec_tracked_planar.size()>0 && vec_tracked_pole.size()>0)
    {
        if(vec_tracked_planar[0].index == vec_tracked_pole[0].index)
        {
            if(first_run)
            {   
                Eigen::Matrix4f pose = cvt::geoPose2eigen(msg->odom);
                Eigen::VectorXf pose_xyz = cvt::eigen2xyzrpy(pose);
                initParticle(cvt::geoPose2eigen(msg->odom),0.2); //2, 5
                std::cout<<"\033[1;34m[MCL]\033[0m callback initpose " << pose_xyz(0) << "," << pose_xyz(1) << "," << pose_xyz(5) * 180 / M_PI << std::endl;
                first_run = false;
            }
            
            selected_poles->points.clear();
            extract_closest_pole(vec_tracked_pole[0], *selected_poles);

            selected_planars.planar.clear();
            scan_planar_prt->points.clear();
            extract_closest_planar(vec_tracked_planar[0], selected_planars);

            Eigen::Matrix4f crt_odom = cvt::geoPose2eigen(vec_tracked_planar[0].odom); // lidar odometry!
            static Eigen::Matrix4f pre_odom = crt_odom;
            m_predictPose = pre_odom.inverse() * crt_odom;
            pre_odom = crt_odom;

            particle best_particle = spinOnce();
            Eigen::VectorXf bestPose_xyz = cvt::eigen2xyzrpy(best_particle.pose);

            // // // //m_pub_mclInfo
            // m_bestpose = best_particle.pose;
            // jslocalization::mcl mclInfo;
            // mclInfo.lidar = cvt::cloud2msg(*closest_points_scan);
            // mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
            // m_pub_mclInfo.publish(mclInfo);

            // // // publish
            pubparticle(best_particle);
            publidar(*selected_poles, cvt::geoPose2eigen(msg->gps_odom));//best_particle.pose);
            pubparticles(m_particles);

            vec_tracked_planar.erase(vec_tracked_planar.begin());
            vec_tracked_pole.erase(vec_tracked_pole.begin());

        }
        else
            return;
    }
    else
        return;

}

void mcl::pole_callback(const jslocalization::PoleTrackBoxes::ConstPtr &msg) 
{   
    vec_tracked_pole.push_back(*msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_pcAll(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<msg->objlist.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pole_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg->objlist[i].pole.pc_pole, *pole_pc);
        *pole_pcAll += *pole_pc;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*pole_pcAll, *pole_transformed, unavlib::cvt::geoPose2eigen(msg->gps_odom));
    m_pub_pole.publish(unavlib::cvt::cloud2msg(*pole_transformed, "odom"));
    // std::cout<<"pole callback : "<< msg->index <<std::endl;

    if(vec_tracked_planar.size()>0 && vec_tracked_pole.size()>0)
    {
        if(vec_tracked_planar[0].index == vec_tracked_pole[0].index)
        {
            if(first_run)
            {   
                Eigen::Matrix4f pose = cvt::geoPose2eigen(msg->odom);
                Eigen::VectorXf pose_xyz = cvt::eigen2xyzrpy(pose);
                initParticle(cvt::geoPose2eigen(msg->odom),0.2); //2, 5
                std::cout<<"\033[1;34m[MCL]\033[0m callback initpose " << pose_xyz(0) << "," << pose_xyz(1) << "," << pose_xyz(5) * 180 / M_PI << std::endl;
                first_run = false;
            }
            
            selected_poles->points.clear();
            extract_closest_pole(vec_tracked_pole[0], *selected_poles);

            selected_planars.planar.clear();
            scan_planar_prt->points.clear();
            extract_closest_planar(vec_tracked_planar[0], selected_planars);

            Eigen::Matrix4f crt_odom = cvt::geoPose2eigen(vec_tracked_planar[0].odom); // lidar odometry!
            static Eigen::Matrix4f pre_odom = crt_odom;
            m_predictPose = pre_odom.inverse() * crt_odom;
            pre_odom = crt_odom;

            particle best_particle = spinOnce();
            Eigen::VectorXf bestPose_xyz = cvt::eigen2xyzrpy(best_particle.pose);

            // // // //m_pub_mclInfo
            // m_bestpose = best_particle.pose;
            // jslocalization::mcl mclInfo;
            // mclInfo.lidar = cvt::cloud2msg(*closest_points_scan);
            // mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
            // m_pub_mclInfo.publish(mclInfo);

            // // // publish
            pubparticle(best_particle);
            publidar(*selected_poles, cvt::geoPose2eigen(msg->gps_odom));//best_particle.pose);
            pubparticles(m_particles);

            vec_tracked_planar.erase(vec_tracked_planar.begin());
            vec_tracked_pole.erase(vec_tracked_pole.begin());

        }
        else
            return;
    }
    else
        return;

#if 0 
    if(!pole_map_received)
        return;

    if(first_run)
    {   
        Eigen::Matrix4f pose = cvt::geoPose2eigen(msg->odom);
        Eigen::VectorXf pose_xyz = cvt::eigen2xyzrpy(pose);
        initParticle(cvt::geoPose2eigen(msg->odom),0.2); //2, 5
        std::cout<<"\033[1;34m[MCL]\033[0m callback initpose " << pose_xyz(0) << "," << pose_xyz(1) << "," << pose_xyz(5) * 180 / M_PI << std::endl;
        first_run = false;
    }
    closest_scan_pole->points.clear();
    extract_closest_feature(*msg, *closest_scan_pole);

    Eigen::Matrix4f crt_odom = cvt::geoPose2eigen(msg->odom); // lidar odometry!
    static Eigen::Matrix4f pre_odom = crt_odom;
    m_predictPose = pre_odom.inverse() * crt_odom;
    pre_odom = crt_odom;

    particle best_particle = spinOnce();

    Eigen::VectorXf bestPose_xyz = cvt::eigen2xyzrpy(best_particle.pose);

    std::cout<<"\033[1;34m[MCL]\033[0m "<<msg->index<<" th node / pose:" << bestPose_xyz(0) << "," << bestPose_xyz(1) << "," << bestPose_xyz(5) * 180 / M_PI << " /#: " << m_particles.size() << std::endl;

    // // //m_pub_mclInfo
    m_bestpose = best_particle.pose;
    jslocalization::mcl mclInfo;
    mclInfo.lidar = cvt::cloud2msg(*closest_scan_pole);
    mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
    m_pub_mclInfo.publish(mclInfo);

    // // publish
    pubparticle(best_particle);
    publidar(*closest_scan_pole, cvt::geoPose2eigen(msg->gps_odom));//best_particle.pose);
    pubparticles(m_particles);
    #endif
}

#if 0
void mcl::pole_callback(const jslocalization::ObjTrackBoxes::ConstPtr &msg) 
{   

    if(!pole_map_received)
        return;

    if(first_run)
    {   
        Eigen::Matrix4f pose = cvt::geoPose2eigen(msg->odom);
        Eigen::VectorXf pose_xyz = cvt::eigen2xyzrpy(pose);
        initParticle(cvt::geoPose2eigen(msg->odom),0.2); //2, 5
        std::cout<<"\033[1;34m[MCL]\033[0m callback initpose " << pose_xyz(0) << "," << pose_xyz(1) << "," << pose_xyz(5) * 180 / M_PI << std::endl;
        first_run = false;
    }
    
    closest_points_scan->points.clear();
    extract_closest_feature(*msg, *closest_points_scan);

    Eigen::Matrix4f crt_odom = cvt::geoPose2eigen(msg->odom); // lidar odometry!
    static Eigen::Matrix4f pre_odom = crt_odom;
    m_predictPose = pre_odom.inverse() * crt_odom;
    pre_odom = crt_odom;

    particle best_particle = spinOnce();

    Eigen::VectorXf bestPose_xyz = cvt::eigen2xyzrpy(best_particle.pose);

    std::cout<<"\033[1;34m[MCL]\033[0m "<<msg->index<<" th node / pose:" << bestPose_xyz(0) << "," << bestPose_xyz(1) << "," << bestPose_xyz(5) * 180 / M_PI << " /#: " << m_particles.size() << std::endl;

    // // //m_pub_mclInfo
    m_bestpose = best_particle.pose;
    jslocalization::mcl mclInfo;
    mclInfo.lidar = cvt::cloud2msg(*closest_points_scan);
    mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
    m_pub_mclInfo.publish(mclInfo);

    // // publish
    pubparticle(best_particle);
    publidar(*closest_points_scan, cvt::geoPose2eigen(msg->gps_odom));//best_particle.pose);
    pubparticles(m_particles);

}
    #endif
void mcl::planar_callback(const jslocalization::PlanarTrackBoxes::ConstPtr &msg)
{
    vec_tracked_planar.push_back(*msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_pcAll(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<msg->objlist.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr planar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg->objlist[i].planar.pc_planar, *planar_pc);
        *planar_pcAll += *planar_pc;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*planar_pcAll, *planar_transformed, unavlib::cvt::geoPose2eigen(msg->gps_odom));
    m_pub_planar.publish(unavlib::cvt::cloud2msg(*planar_transformed, "odom"));
    // std::cout<<"planar callback : "<< msg->index <<std::endl;
}

void mcl::planar_callback(const jslocalization::planars::ConstPtr &msg)
{
#if 0
    if(!planar_map_received)
        return;

    vec_planars.push_back(*msg);

    if(first_run)
    {   
        Eigen::Matrix4f pose = cvt::geoPose2eigen(vec_planars[0].odom);
        Eigen::VectorXf pose_xyz = cvt::eigen2xyzrpy(pose);
        initParticle(cvt::geoPose2eigen(vec_planars[0].odom),0.2); //2, 5
        std::cout<<"\033[1;34m[MCL]\033[0m callback initpose " << pose_xyz(0) << "," << pose_xyz(1) << "," << pose_xyz(5) * 180 / M_PI << std::endl;
        first_run = false;
    }
    
    selected_planars.planar.clear();
    scan_planar_prt->points.clear();
    extract_closest_planar(vec_planars[0], selected_planars);

    Eigen::Matrix4f crt_odom = cvt::geoPose2eigen(vec_planars[0].odom); // lidar odometry!
    static Eigen::Matrix4f pre_odom = crt_odom;
    m_predictPose = pre_odom.inverse() * crt_odom;
    pre_odom = crt_odom;

    particle best_particle = spinOnce();

    Eigen::VectorXf bestPose_xyz = cvt::eigen2xyzrpy(best_particle.pose);

    std::cout<<"\033[1;34m[MCL]\033[0m "<<msg->index<<" th node / pose:" << bestPose_xyz(0) << "," << bestPose_xyz(1) << "," << bestPose_xyz(5) * 180 / M_PI << " /#: " << m_particles.size() << std::endl;

    // // //m_pub_mclInfo
    m_bestpose = best_particle.pose;
    jslocalization::mcl mclInfo;
    // mclInfo.lidar = cvt::cloud2msg(*closest_points_scan);
    mclInfo.mclPose.pose.pose = cvt::eigen2geoPose(m_bestpose);
    m_pub_mclInfo.publish(mclInfo);

    // // publish
    pubparticle(best_particle);
    // publidar(*closest_points_scan, cvt::geoPose2eigen(msg->gps_odom));//best_particle.pose);
    pubparticles(m_particles);

    vec_planars.erase(vec_planars.begin());
    #endif
}

void mcl::extract_closest_planar(jslocalization::PlanarTrackBoxes planars, jslocalization::planars &out_planars)
{   
    pcl::PointCloud<pcl::PointXYZ> pc_scan;

    if(planars.objlist.size()==0){
        std::cout<<"\033[1;34m[MCL]\033[0m THERE IS NO PLANAR!"<<std::endl;
        return;
    }
    
    // for(size_t i=0;i<planars.objlist.size();i++) // tracking하면서 index가 섞여버림 
    // {
    //     planars.objlist[i].planar.index = i;
    // }    

    std::cout<< "\033[1;34m[MCL]\033[0m planars.objlist.size() : "<<planars.objlist.size()<<std::endl;

    for(size_t i=0;i<planars.objlist.size();i++)
    {
        float a = planars.objlist[i].planar.slope;
        float b = planars.objlist[i].planar.start.y - a*planars.objlist[i].planar.start.x;
        float dist2origin = abs(b)/sqrt(pow(a,2)+1);

        pcl::PointXYZ temp;
        temp.x = planars.objlist[i].planar.index;
        temp.y = 0;
        temp.z = planars.objlist[i].planar.length;//dist2origin;
        pc_scan.points.push_back(temp);
        std::cout<<"["<<i<<"] index: "<<planars.objlist[i].planar.index<<" dist: "<<dist2origin<<" length: "<<planars.objlist[i].planar.length<<std::endl;
    }

    sort(pc_scan.points.begin(),pc_scan.end(),planar_cmp); 
    std::cout<<"longest index: "<<pc_scan.points[0].x<<" length: "<<pc_scan.points[0].z<<std::endl;

    int planar_max = 0;
    if(pc_scan.points.size() < m_param_planar_max && pc_scan.points.size() > 0)
    {
        planar_max = pc_scan.points.size();
        std::cout<<"\033[1;34m[MCL]\033[0m\033[1;31m 1111size :  \033[0m"<<planar_max<< std::endl;
    }
    else if(pc_scan.points.size() >= m_param_planar_max)
    {
        planar_max = m_param_planar_max;
         std::cout<<"\033[1;34m[MCL]\033[0m\033[1;31m 2222size :  \033[0m"<<planar_max<< std::endl;
    }
    else
    {
        std::cout<<"\033[1;34m[MCL]\033[0m\033[1;31m THERE IS NO PLANAR!\033[0m" << std::endl;
        return;
    }

    std::cout<<"planar max : "<<planar_max<<" index: "<<pc_scan[0].x<<std::endl;
    for(size_t i =0; i<planar_max; i++){
        jslocalization::planar temp_planar;
        int index = pc_scan[i].x;
        temp_planar = planars.objlist[index].planar;

        out_planars.planar.push_back(temp_planar);
        pcl::PointXYZI temp;
        temp.x = temp_planar.centroid.x;
        temp.y = temp_planar.centroid.y;
        temp.z = 0;
        temp.intensity = temp_planar.theta;
        std::cout<<"["<<i<<"] x: "<<temp.x<<" y: "<<temp.y<<" th: "<<temp.intensity<<" length:"<<temp_planar.length<<std::endl;
        scan_planar_prt->points.push_back(temp);
    }
    std::cout<<"\033[1;34m[MCL]\033[0m Total planar #: "<<planars.objlist.size()<<" selcted planar #: "<<planar_max<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_planar_ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*scan_planar_prt, *scan_planar_ptr_transformed,unavlib::cvt::geoPose2eigen(planars.gps_odom));
    m_pub_scan_planar.publish(unavlib::cvt::cloud2msg(*scan_planar_ptr_transformed, "odom"));

}

void mcl::extract_closest_feature(jslocalization::ObjTrackBoxes poles, pcl::PointCloud<pcl::PointXYZ> &out_cloud)
{
    pcl::PointCloud<pcl::PointXYZI> pc_scan;
    pcl::PointCloud<pcl::PointXYZ> pc_scan_filtered;
    pcl::PointCloud<pcl::PointXYZI> pc_scan_static;

    for(size_t i =0;i<poles.objlist.size();i++)
    {   
        pcl::PointXYZI temp;
        temp.x = (poles.objlist[i].min_x + poles.objlist[i].max_x)/2;
        temp.y = (poles.objlist[i].min_y + poles.objlist[i].max_y)/2;
        temp.z = sqrt(pow(temp.x,2)+pow(temp.y,2));

        float velocity = sqrt(pow(poles.objlist[i].delta_x,2) + pow(poles.objlist[i].delta_y,2));
        temp.intensity = velocity;

        pc_scan.points.push_back(temp);


        std::cout<<"pole ["<<i<<"] velo: "<< velocity<<std::endl;
        if( velocity < m_static_velocity)
        {
            pc_scan_static.points.push_back(temp);
        }
    }  
    
    sort(pc_scan_static.points.begin(),pc_scan_static.end(),point_cmp); 

    int pole_max = 0;
    if(pc_scan_static.points.size() < m_param_planar_max && pc_scan_static.points.size() > 0)
    {
        pole_max = pc_scan_static.points.size();
    }
    else if(pc_scan_static.points.size() >= m_param_pole_max)
    {
        pole_max = m_param_pole_max;
    }
    else
    {
        std::cout<<"\033[1;34m[MCL]\033[0m\033[1;31m THERE IS NO POLE!\033[0m" << std::endl;
        out_cloud = pc_scan_filtered;
        return;
    }

    std::cout<<"\033[1;34m[MCL]\033[0m Total pole #: "<<poles.objlist.size()<<" static pole #: "<< pc_scan_static.points.size()<<" sleected pole #: "<< pole_max<<std::endl;

    for(size_t i =0; i<pole_max; i++){
        pcl::PointXYZ temp;
        temp.x = pc_scan_static.points[i].x;
        temp.y = pc_scan_static.points[i].y;
        temp.z = 0;
        pc_scan_filtered.points.push_back(temp);
        std::cout<<"["<<i<<"] x: "<<temp.x<<" y: "<<temp.y<<" vel: "<<pc_scan_static[i].intensity<<std::endl;
    }
    out_cloud = pc_scan_filtered;

}

void mcl::extract_closest_pole(jslocalization::PoleTrackBoxes poles, pcl::PointCloud<pcl::PointXYZI> &out_cloud)
{
    pcl::PointCloud<pcl::PointXYZI> pc_scan;
    pcl::PointCloud<pcl::PointXYZI> pc_scan_filtered;

    for(size_t i =0;i<poles.objlist.size();i++)
    {   
        pcl::PointXYZI temp;
        temp.x = (poles.objlist[i].min_x + poles.objlist[i].max_x)/2;
        temp.y = (poles.objlist[i].min_y + poles.objlist[i].max_y)/2;
        temp.z =  sqrt(pow(temp.x,2)+pow(temp.y,2)); //sqrt(pow(temp.y,2)); //
        temp.intensity = poles.objlist[i].pole.height;         

        pc_scan.points.push_back(temp);
    }  
    
    sort(pc_scan.points.begin(),pc_scan.end(),point_cmp); 

    int pole_max = 0;
    if(pc_scan.points.size() < m_param_pole_max && pc_scan.points.size() > 0)
    {
        pole_max = pc_scan.points.size();
    }
    else if(pc_scan.points.size() >= m_param_pole_max)
    {
        pole_max = m_param_pole_max;
    }
    else
    {
        std::cout<<"\033[1;34m[MCL]\033[0m\033[1;31m THERE IS NO POLE!\033[0m" << std::endl;
        out_cloud = pc_scan_filtered;
        return;
    }

    std::cout<<"\033[1;34m[MCL]\033[0m Total pole #: "<<poles.objlist.size()<<" sleected pole #: "<< pole_max<<std::endl;

    for(size_t i =0; i<pole_max; i++){
        pcl::PointXYZI temp;
        temp.x = pc_scan.points[i].x;
        temp.y = pc_scan.points[i].y;
        temp.z = 0;
        temp.intensity = pc_scan.points[i].intensity;
        pc_scan_filtered.points.push_back(temp);
    }
    out_cloud = pc_scan_filtered;
}


void mcl::extract_every_feature(jslocalization::ObjTrackBoxes poles, pcl::PointCloud<pcl::PointXYZ> &out_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pc_scan;

    for(size_t i =0;i<poles.objlist.size();i++)
    {   
        pcl::PointXYZ temp;
        temp.x = (poles.objlist[i].min_x + poles.objlist[i].max_x)/2;
        temp.y = (poles.objlist[i].min_y + poles.objlist[i].max_y)/2;
        temp.z = 0;
        pc_scan.points.push_back(temp);
    }  
    out_cloud = pc_scan;

}


void mcl::hdmap_pole_callback(const jslocalization::poles::ConstPtr &map)
{
    if(!pole_map_received){
        poles_map = *map;

        sensor_msgs::PointCloud2::Ptr cloud_in (new sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_raw(new pcl::PointCloud<pcl::PointXYZI>);

        *cloud_in = map->pc_pole;
        pcl::fromROSMsg(*cloud_in, *pc_raw);
        *map_pole_ptr += *pc_raw;

        pole_map_received = true;
        std::cout<<"\033[1;34m[MCL]\033[0m HDmap Pole map data are received"<< std::endl;
    }
}

void mcl::hdmap_planar_callback(const jslocalization::planars::ConstPtr &map)
{
    if(!planar_map_received){
        planars_map = *map;

        sensor_msgs::PointCloud2::Ptr cloud_in (new sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_raw(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(map->pc_planar, *pc_raw);
        *map_planar_ptr += *pc_raw;

        planar_map_received = true;
        std::cout<<"\033[1;34m[MCL]\033[0m HDmap Planar map data are received"<< std::endl;
    }
}


void mcl::data_association_pole(pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud, pcl::PointXYZI observation, pcl::PointXYZI &closest_pole, int &closest_pole_id, float &min_dist)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (map_cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    closest_pole.x = 0;
    closest_pole.y = 0;
    closest_pole.z = 0;
    closest_pole_id = -1;
    min_dist = -1;

    if ( kdtree.nearestKSearch (observation, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {   
        for (std::size_t m = 0; m < pointIdxNKNSearch.size (); ++m)
        {
            closest_pole = (*map_cloud)[pointIdxNKNSearch[m]];
            closest_pole_id = pointIdxNKNSearch[m];
            min_dist = pointNKNSquaredDistance[m];
        }
    }

}

void mcl::data_association_planar(pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud, pcl::PointXYZI observation, pcl::PointXYZI &closest_planar, int &closest_planar_id, float &min_dist)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (map_cloud);
    std::vector<int> pointIdxNKNSearch(5);
    std::vector<float> pointNKNSquaredDistance(5);

    closest_planar.x = 0;
    closest_planar.y = 0;
    closest_planar.z = 0;
    closest_planar.intensity = 0;
    closest_planar_id = -1;
    min_dist = -1;

    pcl::PointCloud<pcl::PointXYZI> pc_candidates;
    
    if ( kdtree.nearestKSearch (observation, 5, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {   
        for (std::size_t m = 0; m < pointIdxNKNSearch.size (); ++m)
        {
            pcl::PointXYZI candidate;
            candidate.x = pointIdxNKNSearch[m]; // id
            candidate.y = map_cloud->points[pointIdxNKNSearch[m]].z; // length

            map_cloud->points[pointIdxNKNSearch[m]].intensity = limitAngle2(map_cloud->points[pointIdxNKNSearch[m]].intensity);
            observation.intensity = limitAngle2(observation.intensity);

            candidate.z = fabs(limitAngle2(map_cloud->points[pointIdxNKNSearch[m]].intensity - observation.intensity)); //theta
            candidate.intensity = pointNKNSquaredDistance[m]; //min_dist ? 
            pc_candidates.points.push_back(candidate);

            std::cout<< "[DA] osv_th: "<<observation.intensity<<" map: "<<map_cloud->points[pointIdxNKNSearch[m]].intensity<<" delta: "<<candidate.z<<" dist: "<< candidate.intensity<<std::endl;
        }

        sort(pc_candidates.points.begin(), pc_candidates.end(), point_cmp);
        std::cout<<"[0] theta: "<<pc_candidates.points[0].z<<std::endl;
        std::cout<<"[1] theta: "<<pc_candidates.points[1].z<<std::endl;
        std::cout<<"[2] theta: "<<pc_candidates.points[2].z<<std::endl;
        std::cout<<"[3] theta: "<<pc_candidates.points[3].z<<std::endl;
        std::cout<<"[4] theta: "<<pc_candidates.points[4].z<<std::endl;

        closest_planar = map_cloud->points[pc_candidates.points[0].x]; 
        closest_planar_id = pc_candidates.points[0].x;
        closest_planar.intensity = map_cloud->points[pc_candidates.points[0].x].intensity;
        min_dist = pc_candidates.points[0].intensity;

        std::cout<<"closest id: "<<closest_planar_id<<" obs: "<<observation.intensity<<" map: "<<closest_planar.intensity<<" delta: "<<pc_candidates.points[0].z<<" min_dist: "<<min_dist<<std::endl;
    }

}

particle mcl::spinOnce()
{
    m_particles = prediction(m_particles, m_predictPose); 

    weightning();  

    particle bestParticle = getBestParticle();

    resampling();

    return bestParticle;
}


void mcl::initParticle(Eigen::Matrix4f initpose, float boundary, float boundary_angle)
{
    Eigen::VectorXf pos_in = cvt::eigen2xyzrpy(initpose);

    std::cout<< "initpose: " << initpose<<std::endl;

    std::default_random_engine gen;
    std::uniform_real_distribution<double> x_pos(pos_in[0] - boundary, pos_in[0] + boundary);
    std::uniform_real_distribution<double> y_pos(pos_in[1] - boundary, pos_in[1] + boundary);
    std::uniform_real_distribution<double> theta_pos(pos_in[5] - boundary_angle * M_PI / 180, pos_in[5] + boundary_angle * M_PI / 180);
    m_particles.clear();
    // weights.clear();
    for(int i=0;i<m_param_particleMax;i++)
    {
      particle particle_temp;
      particle_temp.pose = cvt::xyzrpy2eigen(x_pos(gen),y_pos(gen),0,0,0,theta_pos(gen));
      particle_temp.score = 1/(float)m_param_particleMax;
      m_particles.push_back(particle_temp);
      // weights.push_back(particle_temp.score);
    }
}


std::vector<particle> mcl::prediction(std::vector<particle> particles, Eigen::MatrixXf trans)
{   
    Eigen::VectorXf diff_xyzrpy = cvt::eigen2xyzrpy(trans);
    double delta_trans = sqrt(pow(diff_xyzrpy[0],2) + pow(diff_xyzrpy[1],2));
    double delta_rot1 = atan2(diff_xyzrpy[1], diff_xyzrpy[0]);
    if(delta_trans < 0.001)
        delta_rot1 = 0;
    double delta_rot2 = diff_xyzrpy[5] - delta_rot1;

    if(delta_rot1  > M_PI)
        delta_rot1 -= (2*M_PI);
    if(delta_rot1  < -M_PI)
        delta_rot1 += (2*M_PI);
    if(delta_rot2  > M_PI)
        delta_rot2 -= (2*M_PI);
    if(delta_rot2  < -M_PI)
        delta_rot2 += (2*M_PI);

    double delta_trans_err = m_param_cov[2] * abs(delta_trans) + m_param_cov[3] * abs(delta_rot1 + delta_rot2);
    double delta_rot1_err = m_param_cov[0] * abs(delta_rot1) + m_param_cov[1] * abs(delta_trans);
    double delta_rot2_err = m_param_cov[0] * abs(delta_rot2) + m_param_cov[1] * abs(delta_trans);

    for(size_t i=0;i<particles.size();i++)
    {
        double tilde_trans = delta_trans + probabilistic::GaussianRand() * delta_trans_err;
        double tilde_rot1 = delta_rot1 + probabilistic::GaussianRand() * delta_rot1_err;
        double tilde_rot2 = delta_rot2 + probabilistic::GaussianRand() * delta_rot2_err;

        double tmp_diff_x = tilde_trans * cos(tilde_rot1) + probabilistic::GaussianRand() * m_param_cov[4];
        double tmp_diff_y = tilde_trans * sin(tilde_rot1) + probabilistic::GaussianRand() * m_param_cov[5];
        double tmp_diff_theta = tilde_rot1 + tilde_rot2 + probabilistic::GaussianRand() * m_param_cov[0] * CV_PI / 180;

        particles.at(i).pose= particles.at(i).pose
                * cvt::xyzrpy2eigen(tmp_diff_x,tmp_diff_y,0, 0,0,tmp_diff_theta);
    }
    return particles;
}

#if 0 
// for planar
void mcl::weightning()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_accumulated(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i=0;i<m_particles.size();i++)
    {
        if(selected_planars.planar.size()==0) // 확인 필요! 
            return;
        else
        {
            // Transfom observation from vehicle's coordinate to map coodinate
            pcl::PointCloud<pcl::PointXYZI>::Ptr planar_transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*scan_planar_prt, *planar_transformed, m_particles.at(i).pose); 
            Eigen::VectorXf pose_xyzrpy = cvt::eigen2xyzrpy(m_particles.at(i).pose);

            float weight = 0;

            std::cout<<"selected planars size: "<<selected_planars.planar.size()<<std::endl;
            for(int j=0;j<selected_planars.planar.size();j++)
            {
                pcl::PointXYZI closest_point;
                int closest_id;
                float min_dist;
            
                float theta_robot = limitAngle(pose_xyzrpy(5)*180/M_PI);
                float theta_mearsure = limitAngle(planar_transformed->points[j].intensity);
                float theta_observ = limitAngle(180 + theta_robot + theta_mearsure);
                planar_transformed->points[j].intensity = theta_observ;

                data_association(map_planar_ptr, planar_transformed->points[j], closest_point, closest_id, min_dist);
                map_accumulated->points.push_back(closest_point);


                float theta_delta = abs(theta_observ - closest_point.intensity);
                float weight_angle = limitAngle2(theta_delta) + m_theta_noise;

                std::cout<<"[weight] ["<<j<<"] robot : "<<theta_robot<<" measure: "<<theta_mearsure
                <<" observ: "<<theta_observ<<" map: "<<closest_point.intensity<<" del: "<< weight_angle <<std::endl;


                if(min_dist <0 || closest_id <0 || abs(weight_angle)>  10.0 ) //|| min_dist > 3.0
                {
                    weight += 0;
                }
                else
                {   
                    weight += calcWeight(weight_angle, m_planar_cov);
                }

            }
            weight = weight / planar_transformed->points.size() + 0.00000000001;

            m_particles.at(i).score = m_particles.at(i).score * weight;
        }
    } 

    m_pub_map_planar.publish(cvt::cloud2msg(*map_accumulated, "odom"));
}
#endif 

float mcl::limitAngle2(float input_angle)
{
    if(input_angle  > 180)
        input_angle -= (360);
    if(input_angle  < -180)
        input_angle += (360);
    if(input_angle  > 180)
        input_angle -= (360);
    if(input_angle  < -180)
        input_angle += (360);

    if(input_angle > 90)
        input_angle -= 180;
    if(input_angle < -90)
        input_angle += 180;
    if(input_angle > 90)
        input_angle -= 180;
    if(input_angle < -90)
        input_angle +=180;

    return input_angle;
}

float mcl::limitAngle(float input_angle)
{
    if(input_angle  > 180)
        input_angle -= (360);
    if(input_angle  < -180)
        input_angle += (360);
    if(input_angle  > 180)
        input_angle -= (360);
    if(input_angle  < -180)
        input_angle += (360);

    return input_angle;
}


#if 1
// for pole
void mcl::weightning()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pole_associated_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planar_associated_map(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i=0;i<m_particles.size();i++)
    {
        if(selected_poles->points.size()>0) 
        {
            // Transfom observation from vehicle's coordinate to map coodinate
            pcl::PointCloud<pcl::PointXYZI>::Ptr pole_transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*selected_poles, *pole_transformed, m_particles.at(i).pose); 

            float weight_pole = 0;

            for(size_t j=0;j<pole_transformed->points.size();j++)
            {   
                pcl::PointXYZI closest_point;
                int closest_id;
                float min_dist;

                // Associate obsevation to landmarks using nearest neighbor algorithm
                data_association_pole(map_pole_ptr, pole_transformed->points[j], closest_point, closest_id, min_dist);
                pole_associated_map->points.push_back(closest_point);


                if(min_dist <0 || closest_id <0 || sqrt(min_dist)>  0.5)
                {
                    weight_pole += 0;
                }
                else
                {   
                    weight_pole += calcWeight_pole(min_dist, poles_map.pole[closest_id].covariance);
                }
            }
            weight_pole = weight_pole / pole_transformed->points.size() + 0.00000000001;

            m_particles.at(i).score = m_particles.at(i).score * weight_pole;
        }

        // weights[i] = m_particles.at(i).score;


        if(selected_planars.planar.size()>0) // 확인 필요! 
        {
            // Transfom observation from vehicle's coordinate to map coodinate
            pcl::PointCloud<pcl::PointXYZI>::Ptr planar_transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*scan_planar_prt, *planar_transformed, m_particles.at(i).pose); 
            Eigen::VectorXf pose_xyzrpy = cvt::eigen2xyzrpy(m_particles.at(i).pose);

            float weight_planar = 0;

            std::cout<<"selected planars size: "<<selected_planars.planar.size()<<std::endl;
            for(int j=0;j<selected_planars.planar.size();j++)
            {
                pcl::PointXYZI closest_point;
                int closest_id;
                float min_dist;
            
                float theta_robot = limitAngle(pose_xyzrpy(5)*180/M_PI);
                float theta_mearsure = limitAngle(planar_transformed->points[j].intensity);
                float theta_observ = limitAngle(180 + theta_robot + theta_mearsure);
                planar_transformed->points[j].intensity = theta_observ;

                data_association_planar(map_planar_ptr, planar_transformed->points[j], closest_point, closest_id, min_dist);
                planar_associated_map->points.push_back(closest_point);


                float theta_delta = abs(theta_observ - closest_point.intensity);
                float weight_angle = limitAngle2(theta_delta) + m_theta_noise;

                std::cout<<"[weight] ["<<j<<"] robot : "<<theta_robot<<" measure: "<<theta_mearsure
                <<" observ: "<<theta_observ<<" map: "<<closest_point.intensity<<" del: "<< weight_angle <<std::endl;


                if(min_dist <0 || closest_id <0 || abs(weight_angle)>  10.0 ) //|| min_dist > 3.0
                {
                    weight_planar += 0;
                }
                else
                {   
                    weight_planar += calcWeight_planar(weight_angle, m_planar_cov);
                }

            }
            weight_planar = weight_planar / planar_transformed->points.size() + 0.00000000001;

            m_particles.at(i).score = m_particles.at(i).score * weight_planar;
        }
    }

    m_pub_map_pole.publish(cvt::cloud2msg(*pole_associated_map, "odom"));
    m_pub_map_planar.publish(cvt::cloud2msg(*planar_associated_map, "odom"));

}

float mcl::calcWeight_pole(float delta_dist, float covariance)
{
    return (float) exp(-delta_dist/(2*covariance)); 
}

float mcl::calcWeight_planar(float delta_ang, float covariance)
{
    return (float) exp(-abs(delta_ang)/(2*covariance)); 
    // return (float) exp(-pow(delta_ang*M_PI/180,2)/(2*pow(covariance,2))) / sqrt(2*M_PI*covariance); 
}

#endif


#if 0
void mcl::resampling()
{   
    static int cnt = 0;
    std::string filePath1 = "/home/jsg/current/landmark_ws/weights/weights";
    filePath1 += std::to_string(cnt) +".txt";
    std::ofstream writeFile1(filePath1.data());

    std::string filePath2 = "/home/jsg/current/landmark_ws/weights/resampled_weights";
    filePath2 += std::to_string(cnt)+".txt";
    std::ofstream writeFile2(filePath2.data());
    cnt++;

    float particles_sumscore = 0;
    for(size_t i=0;i<m_particles.size();i++){
        particles_sumscore += m_particles.at(i).score;
    }

    std::vector<float> particles_scores;
    float normalized_scores = 0;
    for(size_t i=0;i<m_particles.size();i++)
    {
      m_particles.at(i).score = m_particles.at(i).score / particles_sumscore; // This is for preventing specific particle to have too much score.
      weights[i] = m_particles.at(i).score;
      if(writeFile1.is_open()){
        writeFile1<<weights[i]<<" "<<m_particles.at(i).pose(0,3)<<" "<<m_particles.at(i).pose(1,3)<<std::endl;
      }
    }
    // writeFile1.close();

    int particleNum = m_particles.size();

    std::default_random_engine gen;    
    std::vector<particle> resampled_particles;
    std::vector<double> resampled_weights; 

    //Generate random particle index
    std::uniform_int_distribution<int> particle_index(0, particleNum - 1);
    
    int current_index = particle_index(gen);

    double beta = 0.0;  
    double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());

    for (size_t i = 0; i < particleNum; i++) 
    {

        std::uniform_real_distribution<double> random_weight(0.0, max_weight_2);
        beta += random_weight(gen);

        // std::cout<<"current_index: "<<current_index<<" beta: "<<beta<<std::endl;

        while (beta > weights[current_index]) 
        {
            beta -= weights[current_index];
            current_index = (current_index + 1) % particleNum;
            // std::cout<< "while beta: "<<beta<<" current_index: "<< current_index<<" current weights: "<<weights[current_index]<<std::endl;
        }   
        // std::cout<<"num_particles: "<<m_particles.size()<<" current index: "<<current_index<<" weight: " <<weights[current_index]<<std::endl;
        resampled_particles.push_back(m_particles.at(current_index));
        resampled_weights.push_back(weights[current_index]);

        if(writeFile2.is_open()){
            writeFile2<<weights[current_index]<<" "<<m_particles.at(current_index).pose(0,3)<<" "<<m_particles.at(current_index).pose(1,3)<<std::endl;
        }
    }
    m_particles = resampled_particles;
    weights = resampled_weights;
    writeFile2.close();
}

void ParticleFilter::Resampling()
{
  std::cout<<"Resampling..." << endl; //<<m_sync_count<<std::endl;

  //Make score line (roullette)
  std::vector<double> particleScores;
  std::vector<Particle> particleSampled;
  double scoreBaseline = 0;
  for(int i=0;i<particles.size();i++)
  {
    scoreBaseline += particles.at(i).weight;
    particleScores.push_back(scoreBaseline);
  }

  std::uniform_real_distribution<double> dart(0, scoreBaseline);
  for(int i=0;i<particles.size();i++)
  {
    double darted = dart(gen); //darted number. (0 to maximum scores)
    auto lowerBound = std::lower_bound(particleScores.begin(), particleScores.end(), darted);
    int particleIndex = lowerBound - particleScores.begin(); // Index of particle in particles.

    //TODO : put selected particle to array 'particleSampled' with score reset.

    Particle selectedParticle = particles.at(particleIndex); // Which one you have to select?

    particleSampled.push_back(selectedParticle);

    //-----------------------------------------------------------------------//

  }
  particles = particleSampled;
}

#endif

#if 1
void mcl::resampling()
{
    #if SAVE_PARTICLES
        static int cnt = 0;
        std::string filePath1 = "/home/jsg/current/landmark_ws/weights/weights";
        filePath1 += std::to_string(cnt) +".txt";
        std::ofstream writeFile1(filePath1.data());

        std::string filePath2 = "/home/jsg/current/landmark_ws/weights/resampled_weights";
        filePath2 += std::to_string(cnt)+".txt";
        std::ofstream writeFile2(filePath2.data());
        cnt++;
    #endif


    std::default_random_engine gen;

    float particles_sumscore = 0;
    for(int i=0;i<m_particles.size();i++)
    {
        particles_sumscore += m_particles.at(i).score;
    }

    std::vector<float> particles_scores;
    float normalized_scores = 0;
    for(size_t i=0;i<m_particles.size();i++){
        m_particles.at(i).score = m_particles.at(i).score / particles_sumscore; // This is for preventing specific particle to have too much score.
        normalized_scores += m_particles.at(i).score;
        particles_scores.push_back(normalized_scores);
        // weights[i] = m_particles.at(i).score;
        #if SAVE_PARTICLES
            if(writeFile1.is_open()){
                writeFile1<<m_particles.at(i).score<<" "<<m_particles.at(i).pose(0,3)<<" "<<m_particles.at(i).pose(1,3)<<std::endl;
            }
        #endif
    }

    #if SAVE_PARTICLES
        writeFile1.close();
    #endif

    // std::vector<double> resampled_weights; 

    std::uniform_real_distribution<double> dart(0, particles_scores.back());
    std::vector<particle> particles_sampled;
    int particleNum = adapted_quantity();

    for(size_t i=0;i<particleNum;i++)
    {
        bool particle_selected = false;
        while(!particle_selected)
        {
            double darted = dart(gen);
            for(size_t j=0;j<particles_scores.size();j++)
            {
                if(darted<particles_scores.at(j))
                {   
                    particles_sampled.push_back(m_particles.at(j));
                    // resampled_weights.push_back(m_particles.at(j).score);
                    particle_selected = true;
                    
                    #if SAVE_PARTICLES
                        if(writeFile2.is_open()){
                            writeFile2<<m_particles.at(j).score<<" "<<m_particles.at(j).pose(0,3)<<" "<<m_particles.at(j).pose(1,3)<<std::endl;
                        }
                    #endif

                    break;             
                }
            }
        }
    }
    m_particles = particles_sampled;
    // weights = resampled_weights;

    #if SAVE_PARTICLES 
        writeFile2.close();
    #endif

}
#endif

bool mcl::isOnmap(Eigen::MatrixXf pose)
{}

// particle mcl::getBestParticle()
// {
//     #if SAVE_PARTICLES
//         static int cnt_b = 0;
//         std::string filePath3 = "/home/jsg/current/landmark_ws/weights/best";
//         filePath3 += std::to_string(cnt_b) +".txt";
//         std::ofstream writeFile3(filePath3.data());
//         cnt_b++;
//     #endif

//     particle max_particle;

//     sort(m_particles.begin(),m_particles.end(), particle_cmp); 

//     float sum_x=0, sum_y=0, sum_theta=0;

//     for(size_t i=0; i<3; i++){

//         Eigen::VectorXf pose_xyzrpy = cvt::eigen2xyzrpy(m_particles.at(i).pose);
//         max_particle.score += m_particles.at(i).score/3;
//         sum_x += pose_xyzrpy(0)/3;
//         sum_y += pose_xyzrpy(1)/3;
//         sum_theta += pose_xyzrpy(5)/3;

//         #if SAVE_PARTICLES
//             if(writeFile3.is_open()){
//                 writeFile3<<m_particles[i].score<<" "<<pose_xyzrpy(0)<<" "<<pose_xyzrpy(1)<<std::endl;
//             }
//         #endif
//     }

//     #if SAVE_PARTICLES
//         writeFile3.close();
//     #endif

//     max_particle.pose = cvt::xyzrpy2eigen(sum_x,sum_y,0,0,0,sum_theta);
//     std::cout << "max: " << max_particle.score<< " #: "<< (int)m_particles.size()*0.03<< " " << m_particles[0].score << " " << m_particles[1].score <<" "<<m_particles[2].score<<std::endl;

//     return max_particle;
// }

particle mcl::getBestParticle()
{
    particle max_particle;
    max_particle.score = std::numeric_limits<float>::min();

    for(int i=0;i<m_particles.size();i++)
    {
        if(m_particles.at(i).score > max_particle.score)
        {
            max_particle = m_particles.at(i);
        }
    }
    return max_particle;
}

int mcl::adapted_quantity()
{
    float max_x,max_y,min_x,min_y;
    max_x = std::numeric_limits<float>::min(); max_y = std::numeric_limits<float>::min();
    min_x = std::numeric_limits<float>::max(); min_y = std::numeric_limits<float>::max();
    for(size_t i=0;i<m_particles.size();i++)
    {
        Eigen::VectorXf pose_xyzrpy = cvt::eigen2xyzrpy(m_particles.at(i).pose);
        if(pose_xyzrpy[0]>max_x) max_x = pose_xyzrpy[0];
        if(pose_xyzrpy[0]<min_x) min_x = pose_xyzrpy[0];
        if(pose_xyzrpy[1]>max_y) max_y = pose_xyzrpy[1];
        if(pose_xyzrpy[1]<min_y) min_y = pose_xyzrpy[1];
    }
    
    if( (fabs(max_x-min_x)*fabs(max_y-min_y)*100) > m_param_particleMax ) 
    {
        return m_param_particleMax;
    }
    else if( (fabs(max_x-min_x)*fabs(max_y-min_y)*100) < (m_param_particleMin) )
    {
      return m_param_particleMin;  
    } 
    else{
        return ( fabs(max_x-min_x)*fabs(max_y-min_y)*100 );
    }
}

void mcl::pubparticle(particle particleo)
{
    nav_msgs::Odometry msg_particle;
    msg_particle.header.frame_id="odom";
    msg_particle.pose.pose = cvt::eigen2geoPose(particleo.pose);
    m_pub_mclPose.publish(msg_particle);
}

void mcl::pubparticles(std::vector<particle> particles)
{
    pcl::PointCloud<pcl::PointXYZ> particlePts;
    for(int i=0;i<particles.size();i++)
    {
        pcl::PointXYZ tmp_xyz;
        tmp_xyz.x = particles.at(i).pose(0,3);
        tmp_xyz.y = particles.at(i).pose(1,3);
        tmp_xyz.z = 0;
        particlePts.push_back(tmp_xyz);
    }

    m_pub_mclParticles.publish(cvt::cloud2msg(particlePts, "odom"));
}

void mcl::publidar(pcl::PointCloud<pcl::PointXYZ> lidar, Eigen::Matrix4f pose)
{
    pcl::PointCloud<pcl::PointXYZ> lidar_global;
    pcl::transformPointCloud(lidar,lidar_global,pose);
    sensor_msgs::PointCloud2 msg_lidar = cvt::cloud2msg(lidar_global);
    msg_lidar.header.frame_id="odom";
    m_pub_mclLiDAR.publish(msg_lidar);
}

void mcl::publidar(pcl::PointCloud<pcl::PointXYZI> lidar, Eigen::Matrix4f pose)
{
    pcl::PointCloud<pcl::PointXYZI> lidar_global;
    pcl::transformPointCloud(lidar,lidar_global,pose);
    sensor_msgs::PointCloud2 msg_lidar = cvt::cloud2msg(lidar_global);
    msg_lidar.header.frame_id="odom";
    m_pub_mclLiDAR.publish(msg_lidar);
}

