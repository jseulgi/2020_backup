#include "dynamic_planning/dynamic_planning.h"

DynamicPlanning::DynamicPlanning()
: nh_(),
private_nh_("~"),
pose_initialized_(false),
trajectory_initialized_(false),
node_initialized_(false),
map_initialized_(false),
dynamic_initialized_(false),
path_following_initialized_(false),
terminate_thread_(false)
{
  /* time-related */
  private_nh_.param<int>("loop_rate", loop_rate_, 100);

  /* topics and frame_ids */
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("gps_topic", gps_topic_, "gps_odom");

  /* collision type */
  private_nh_.param<bool>("is_rect", is_rect_, false);
  private_nh_.param<bool>("is_ellipse", is_ellipse_, true);

  /* Trajectory */
  private_nh_.param<int>("path_size", path_size_, 4);
  private_nh_.param<double>("locate_range", locate_range_, 5.0);
  private_nh_.param<double>("offset", offset_, 0.5);
  private_nh_.param<double>("lat_acc_limit", lat_acc_limit_, 5);
  private_nh_.param<double>("safety_gain", safety_gain_, 0.8);
  private_nh_.param<double>("ref_speed", ref_speed_, 13.9);
  private_nh_.param<double>("sign_limit", sign_limit_, 50.0);
  private_nh_.param<double>("ws", ws_, 1.0);
  private_nh_.param<double>("wc", wc_, 1.0);
  private_nh_.param<double>("wl", wl_, 1.0);
  private_nh_.param<double>("a", a_, 1.0);
  private_nh_.param<double>("b", b_, 1.0);
  private_nh_.param<double>("obs_thres", obs_thres_, 1.5);
  private_nh_.param<double>("lane_thres", lane_thres_, 1.5);
  private_nh_.param<double>("cut_in_thres", cut_in_thres_, 50.0);
  private_nh_.param<double>("sigma", sigma_, 0.5);
  private_nh_.param<double>("obs_cost", obs_cost_, 10.0);
  private_nh_.param<bool>("switch", switch_, false);
  private_nh_.param<double>("dist_coeff", dist_coeff_, 1.0);
  private_nh_.param<double>("min_dist", min_dist_, 15.0);
  private_nh_.param<double>("max_dist", max_dist_, 45.0);
  private_nh_.param<double>("stop_dist", stop_dist_, 8.0);

  /* ACC */
  private_nh_.param<double>("max_decel", max_decel_, -3.0);
  private_nh_.param<double>("braking_distance", braking_distance_, 20.0);
  private_nh_.param<double>("stop_push", stop_push_, 10.0);
  private_nh_.param<double>("red_push", red_push_, 5.0);
  private_nh_.param<double>("wait_time_thres", wait_time_thres_, 3.0);

  /* avoidance */
  private_nh_.param<double>("avoidance_wait_time_thres", avoidance_wait_time_thres_, 10.0);
  private_nh_.param<double>("light_thres", light_thres_, 10.0);
  private_nh_.param<double>("turn_thres", turn_thres_, 10.0);
  private_nh_.param<double>("goal_thres", goal_thres_, 10.0);
  /* Car specification */
  private_nh_.param<double>("length", length_, 4.300);
  private_nh_.param<double>("width", width_, 1.795);
  private_nh_.param<double>("gps_to_hood", gps_to_hood_, 2.5);
  private_nh_.param<double>("gps_to_rear", gps_to_rear_, 1.0);

  /* Subscriber */
	current_odom_sub_ = nh_.subscribe(gps_topic_, 1, &DynamicPlanning::currentOdomCallback, this);
  global_trajectory_sub_ = nh_.subscribe("/global_trajectory", 1, &DynamicPlanning::trajectoryCallback, this);
  map_info_sub_ = nh_.subscribe("/map_info", 1, &DynamicPlanning::mapInfoCallback, this);
  object_info_sub_ = nh_.subscribe("/obj_info",1, &DynamicPlanning::vehCallback, this);
  traffic_info_sub_ = nh_.subscribe("/traffic_info", 1, &DynamicPlanning::trafficCallback, this);
  ego_state_sub_ = nh_.subscribe("/vehicle_state", 1, &DynamicPlanning::egoStateCallback, this);
  lookahead_sub_ = nh_.subscribe("/lookahead_distance", 1, &DynamicPlanning::lookaheadCallback, this);
  atypical_obj_sub_ = nh_.subscribe("/two_static_obstacles", 1, &DynamicPlanning::atypicalObjCallback, this);

  // /two_static_obstacles

  /* Publisher */
  ego_speed_pub_ = nh_.advertise<std_msgs::Float64>("/Target_Velocity", 1);
  ego_acc_pub_ = nh_.advertise<std_msgs::Float64>("/Acceleration", 1);
  ego_location_pub_ = nh_.advertise<obj_msgs::Ego>("/ego_location", 1);
  rviz_current_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("/rviz/current_pose", 1);
  rviz_goal_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/rviz/goal_point", 1);
  ref_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/ref_path", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/bounding_box", 1);
  rviz_ellipse_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/ellipse", 1);
  rviz_velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/global_vel", 1);
  rviz_stop_nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/stop_nodes", 1);
  road_state_pub_ = nh_.advertise<path_msgs::State>("/road_state", 1);

  for(int i = 0; i < path_size_*2 + 1; i++)
  {
    std::string topic_name = "/rviz/candidate";
    publishers_.push_back(nh_.advertise<visualization_msgs::Marker>(topic_name + std::to_string(i),1));
  }

  // Initilization
  is_blocked_ = false;
  is_avoiding_ = false;
  first_sight_ = false;
  look_further_ = false;
  count_ = 0;
  ego_speed_ = 0.0;
  ego_acc_ = 0.0;
  static_timer_ = 0.0;
  node_initialized_ = true;
  // prev_obj_id_ = 0;
  // dt_ = 0.1;


  rate_ = new ros::Rate(loop_rate_);
}

DynamicPlanning::~DynamicPlanning()
{
  // publish_thread_.join();
}

void DynamicPlanning::transformAtypicalObject(obj_msgs::Obj &obj)
{
  // Initialize
  std::vector<int> to_indices(4);
  std::vector<int> from_indices(4);
  int x = 0;
  std::iota(to_indices.begin(), to_indices.end(), x++);
  x = 0;
  std::iota(from_indices.begin(), from_indices.end(), x++);
  std::vector<double> from_distances(4);
  std::vector<double> to_distances(4);
  std::vector<geometry_msgs::Point> vertices(4);
  path_msgs::Node from_node;
  path_msgs::Node to_node;

  if (obj.link_id != "")
  {
    path_msgs::Link link = links_.at(obj.link_id);
    from_node = nodes_.at(link.from_node);
    to_node = nodes_.at(link.to_node);
    from_node.geometry.x -= offsetX_;
    from_node.geometry.y -= offsetY_;
    to_node.geometry.x -= offsetX_;
    to_node.geometry.y -= offsetY_;
  }

  vertices[0] = obj.br;
  vertices[1] = obj.bl;
  vertices[2] = obj.ul;
  vertices[3] = obj.ur;
  for (int i = 0; i < 4; i++)
  {
    to_distances[i] = getDistance(to_node.geometry.x, to_node.geometry.y, vertices[i].x, vertices[i].y);
    from_distances[i] = getDistance(from_node.geometry.x, from_node.geometry.y, vertices[i].x, vertices[i].y);
  }
  sort(to_indices.begin(), to_indices.end(), [&](int i, int j) {return to_distances[i] < to_distances[j];});
  sort(from_indices.begin(), from_indices.end(), [&](int i, int j) {return from_distances[i] < from_distances[j];});

  for (int i = 0; i < 4; i++)
  {
    if (vertices[to_indices[0]].x == vertices[i].x)
    {
      printf("%d: to_node: 1st\n", i);
    }
    else if (vertices[to_indices[1]].x == vertices[i].x)
    {
      printf("%d: to_node: 2nd\n", i);
    }
    else if (vertices[to_indices[2]].x == vertices[i].x)
    {
      printf("%d: to_node: 3rd\n", i);
    }
    else if (vertices[to_indices[3]].x == vertices[i].x)
    {
      printf("%d: to_node: 4th\n", i);
    }

    if (vertices[from_indices[0]].x == vertices[i].x)
    {
      printf("%d: from_node: 1st\n", i);
    }
    else if (vertices[from_indices[1]].x == vertices[i].x)
    {
      printf("%d: from_node: 2nd\n", i);
    }
    else if (vertices[from_indices[2]].x == vertices[i].x)
    {
      printf("%d: from_node: 3rd\n", i);
    }
    else if (vertices[from_indices[3]].x == vertices[i].x)
    {
      printf("%d: from_node: 4th\n", i);
    }
  }

}

void DynamicPlanning::atypicalObjCallback(const obj_msgs::ObjList msg)
{
  if (trajectory_initialized_)
  {
    atypical_obj_list_.clear();
    printf("Atypical object received\n");
    for(int i=0; i < msg.objlist.size(); i++)
    {
      obj_msgs::Obj obj = msg.objlist[i];
      // obj.ul.x -= offsetX_;
      // obj.ul.y -= offsetY_;
      // obj.ur.x -= offsetX_;
      // obj.ur.y -= offsetY_;
      // obj.bl.x -= offsetX_;
      // obj.bl.y -= offsetY_;
      // obj.br.x -= offsetX_;
      // obj.br.y -= offsetY_;
      obj.bl.x = -60.095;
      obj.bl.y = 3.97217;
      obj.ul.x = -61.0067;
      obj.ul.y = 1.10881;
      obj.ur.x = -46.3657;
      obj.ur.y = -0.697612;
      obj.br.x = -45.3437;
      obj.br.y = 2.16499;
      obj.link_id = "155M0102ZZ0204";
      obj.type = OType::ATYPICAL;
      obj.pose.position.x = (obj.ul.x + obj.ur.x + obj.bl.x + obj.br.x) / 4.0;
      obj.pose.position.y = (obj.ul.y + obj.ur.y + obj.bl.y + obj.br.y) / 4.0;
      obj.id = -(1000 + i);
      atypical_obj_list_.push_back(obj);
    }
  }
}

void DynamicPlanning::lookaheadCallback(const std_msgs::Float64 msg)
{
  lookahead_distance_ = msg.data;
  path_following_initialized_ = true;
}

void DynamicPlanning::egoStateCallback(const control_msgs::VehicleState msg)
{
  ego_speed_ = msg.v_ego;
  ego_acc_ = msg.a_x;
  //
  // tf::Vector3 local_vel;
  // tf::Transform heading;
  // local_vel.setValue(ego_speed_*1000.0/3600.0, 0, 0);
  // heading.setRotation(tf::Quaternion(ego_.pose.orientation.x,
  //                                     ego_.pose.orientation.y,
  //                                     ego_.pose.orientation.z,
  //                                     ego_.pose.orientation.w));
  // ego_vel_ = heading * local_vel;
  // printf("x: %f, y: %f, z: %f, w: %f\n", ego_.pose.orientation.x, ego_.pose.orientation.y, ego_.pose.orientation.z, ego_.pose.orientation.w);
  // printf("Ego speed: %f, Ego velocity: %f, %f\n", ego_speed_, ego_vel_.x(), ego_vel_.y());
}

void DynamicPlanning::trafficCallback(const obj_msgs::ObjList msg)
{
  traffic_list_.clear();
  // dt_ = 0.1;
  if (trajectory_initialized_)
  {
    for(int i=0; i < msg.objlist.size(); i++)
    {
      obj_msgs::Obj traffic_light = msg.objlist[i];
      traffic_light.type = OType::RED; // RED
      traffic_light.id = -(2000 + i);
      traffic_light.is_wait = true;
      locateObj(traffic_light);
      if (traffic_light.index >= 0 && traffic_light.link_id != "")
      {
        traffic_list_.push_back(traffic_light);
      }
    }
  }
}

void DynamicPlanning::vehCallback(const obj_msgs::ObjList msg)
{
  veh_list_.clear();
  raw_veh_list_.clear();
  ellipse_markers_.markers.clear();
  visualization_msgs::MarkerArray velocity_markers;
  // printf("Raw veh size: %d\n", msg.raw_objlist.size());
  // printf("veh size: %d\n", msg.objlist.size());

  // Iterate through raw vehicles
  for (int i = 0; i < msg.raw_objlist.size(); i++)
  {
    obj_msgs::Obj obj = msg.raw_objlist[i];
    obj_msgs::Obj gobj = obj;
    transformObject(obj, gobj);
    if (trajectory_initialized_)
    {
      locateObj(gobj);
      if (gobj.index >= 0 && gobj.link_id != "")
      {
        raw_veh_list_.push_back(gobj);
      }
    }
  }

  // Iterate Through vehicles with boxes
  for(int i=0; i < msg.objlist.size(); i++)
  {
    if (msg.objlist[i].type != OType::NOINTEREST && msg.objlist[i].type != OType::OPPOSITE)
    // if (msg.objlist[i].type != OType::OPPOSITE)
    {
      obj_msgs::Obj lobj = msg.objlist[i];
      // printf("Input obj %d, type: %d\n", lobj.id, lobj.type);
      if (lobj.pose.position.x == 0.0 && lobj.pose.position.y == 0.0)
        continue;

      obj_msgs::Obj gobj = lobj;
      transformObject(lobj, gobj);
      // visualization_msgs::Marker velocity_marker;
      // velocity_marker.header.frame_id = map_frame_;
      // velocity_marker.header.stamp = ros::Time::now();
      // velocity_marker.id = -2000 + i;
      // velocity_marker.type = visualization_msgs::Marker::ARROW;
      // velocity_marker.action = visualization_msgs::Marker::ADD;
      // velocity_marker.lifetime = ros::Duration(0.1);
      // velocity_marker.points.resize(2);
      // velocity_marker.points[0].x = gobj.pose.position.x;
      // velocity_marker.points[0].y = gobj.pose.position.y;
      // velocity_marker.points[0].z = gobj.pose.position.z;
      // velocity_marker.points[1].x = gobj.pose.position.x + gobj.velocity.x;// * 10.0;///delta_time;
      // velocity_marker.points[1].y = gobj.pose.position.y + gobj.velocity.y;// * 10.0;///delta_time;
      // velocity_marker.points[1].z = gobj.pose.position.z + gobj.velocity.z;
      // velocity_marker.pose.orientation.w = 1.0;
      // velocity_marker.scale.x = 0.5;
      // velocity_marker.scale.y = 0.5;
      // velocity_marker.scale.z = 0;
      // velocity_marker.color.r = 1.0;
      // velocity_marker.color.b = 0.5;
      // velocity_marker.color.a = 1.0;
      // velocity_markers.markers.push_back(velocity_marker);

      if (trajectory_initialized_)
      {
        locateObj(gobj);
        // printf("gobj id: %d\n", gobj.id);
        if (gobj.index >= 0 && gobj.link_id != "")
        {
          obj_msgs::Obj fake = gobj;
          geometry_msgs::Point center;
          gobj = getEllipse(gobj, gobj.pose.position, 1.0);
          // move fake's center to midway between upper side of the box
          if (gobj.type != OType::STATIC)
          {
            center.x = (gobj.ul.x + gobj.ur.x) / 2.0;
            center.y = (gobj.ul.y + gobj.ur.y) / 2.0;
            fake = getEllipse(fake, center, 1.0);
          }
          // move fake's center to x m and size up
          else
          {
            if (abs(gobj.lat_offset) <= HALF_LANE)
            {
              int push_index = gobj.index;
              for(int inc = 0; inc <= 6; inc++)
              {
                if (push_index >= global_trajectory_.waypoints.size())
                {
                  break;
                }
                push_index++;
              }
              center.x = global_trajectory_.waypoints[push_index].point.x;
              center.y = global_trajectory_.waypoints[push_index].point.y;
              fake = getEllipse(fake, center, 2.0);
            }
            else
            {
              center.x = (gobj.ul.x + gobj.ur.x) / 2.0;
              center.y = (gobj.ul.y + gobj.ur.y) / 2.0;
              fake = getEllipse(fake, center, 1.0);
            }
          }
          visualizeEllipse(gobj);
          visualizeEllipse(fake);
          veh_list_.push_back(gobj);
          veh_list_.push_back(fake);
        }
        else
        {
          printf("obj: %d not found\n", gobj.id);
        }
      }
    }
  }

  // Add atypical objects here
  for(int i = 0; i < atypical_obj_list_.size(); i++)
  {
    obj_msgs::Obj obj = atypical_obj_list_[i];
    locateObj(obj);
    if (obj.index >= 0 && obj.link_id != "")
    {
      printf("Before -- ulx: %f, uly: %f, brx: %f, bry: %f \n", obj.ul.x, obj.ul.y, obj.br.x, obj.br.y);
      // if located in the global
      transformAtypicalObject(obj);
      printf("After -- ulx: %f, uly: %f, brx: %f, bry: %f \n", obj.ul.x, obj.ul.y, obj.br.x, obj.br.y);
      obj = getEllipse(obj, obj.pose.position, 1.0);
      visualizeEllipse(obj);
      veh_list_.push_back(obj);
    }
  }

  // add rviz ellipse markers
  rviz_ellipse_pub_.publish(ellipse_markers_);
  rviz_velocity_pub_.publish(velocity_markers);
}

obj_msgs::Obj DynamicPlanning::getEllipse(obj_msgs::Obj obj, geometry_msgs::Point center, double scale)
{
  obj_msgs::Obj ellipse = obj;
  geometry_msgs::Point upper_mid;
  geometry_msgs::Point bottom_mid;
  geometry_msgs::Point right_mid;
  geometry_msgs::Point left_mid;
  upper_mid.x = (obj.ul.x + obj.ur.x) / 2.0;
  upper_mid.y = (obj.ul.y + obj.ur.y) / 2.0;
  bottom_mid.x = (obj.bl.x + obj.br.x) / 2.0;
  bottom_mid.y = (obj.bl.y + obj.br.y) / 2.0;
  right_mid.x = (obj.ur.x + obj.br.x) / 2.0;
  right_mid.y = (obj.ur.y + obj.br.y) / 2.0;
  left_mid.x = (obj.ul.x + obj.bl.x) / 2.0;
  left_mid.y = (obj.ul.y + obj.bl.y) / 2.0;
  double ub_axis = getDistance(upper_mid.x, upper_mid.y, bottom_mid.x, bottom_mid.y) / 2.0;
  double lr_axis = getDistance(right_mid.x, right_mid.y, left_mid.x, left_mid.y) / 2.0;
  //
  // double angle;
  // double a;
  // double b;
  // double d_axis;
  // double d_angle;
  //
  // // Enlarge
  // if (ub_axis >= lr_axis)
  // {
  //   a = ub_axis;
  //   b = lr_axis;
  //   std::vector<double> UM {upper_mid.x - obj.pose.position.x, upper_mid.y - obj.pose.position.y};
  //   std::vector<double> UL {obj.ul.x - obj.pose.position.x, obj.ul.y - obj.pose.position.y};
  //   double dot = std::inner_product(UM.begin(), UM.end(), UL.begin(), 0);
  //   double norm_UM = sqrt(UM[0] * UM[0] + UM[1] * UM[1]);
  //   double norm_UL = sqrt(UL[0] * UL[0] + UL[1] * UL[1]);
  //   d_angle = acos(dot / (norm_UM * norm_UL));
  //   d_axis = getDistance(obj.ul.x, obj.ul.y, obj.pose.position.x, obj.pose.position.y);
  //   angle = atan2(upper_mid.y - obj.pose.position.y, upper_mid.x - obj.pose.position.x);
  // }
  // else
  // {
  //   a = lr_axis;
  //   b = ub_axis;
  //   std::vector<double> UM {right_mid.x - obj.pose.position.x, right_mid.y - obj.pose.position.y};
  //   std::vector<double> UL {obj.ur.x - obj.pose.position.x, obj.ur.y - obj.pose.position.y};
  //   double dot = std::inner_product(UM.begin(), UM.end(), UL.begin(), 0);
  //   double norm_UM = sqrt(UM[0] * UM[0] + UM[1] * UM[1]);
  //   double norm_UL = sqrt(UL[0] * UL[0] + UL[1] * UL[1]);
  //   d_angle = acos(dot / (norm_UM * norm_UL));
  //   d_axis = getDistance(obj.ur.x, obj.ur.y, obj.pose.position.x, obj.pose.position.y);
  //   angle = atan2(right_mid.y - obj.pose.position.y, right_mid.x - obj.pose.position.x);
  // }
  // double b_prime = b + width_ / 2.0;
  // double d_axis_prime = d_axis + width_ / 2.0;
  // double a_prime = d_axis_prime / b_prime * sqrt(b_prime*cos(d_angle)*b*cos(d_angle) + a*sin(d_angle)*a*sin(d_angle));
  // printf("d_angle: %f, d_axis: %f\n", d_angle, d_axis);

  // Enlarge
  double a;
  double b;
  double a_prime;
  double b_prime;
  double angle;

  if (ub_axis >= lr_axis)
  {
    printf("vertically long\n");
    a = ub_axis;
    b = lr_axis;
    b_prime = b + width_/2.0;
    a_prime = a / b * b_prime;
    a_prime *= scale;
    angle = atan2(upper_mid.y - obj.pose.position.y, upper_mid.x - obj.pose.position.x);
  }
  else
  {
    printf("horizontally long\n");
    a = lr_axis;
    b = ub_axis;
    a_prime = a + width_ / 2.0;
    b_prime = b / a * a_prime;
    b_prime *= scale;
    angle = atan2(right_mid.y - obj.pose.position.y, right_mid.x - obj.pose.position.x);
  }
  printf("id: %d, a: %f, b: %f, a': %f, b': %f, angle: %f\n", obj.id, a, b, a_prime, b_prime, angle);

  ellipse.a = a_prime;
  ellipse.b = b_prime;
  ellipse.alpha = angle;
  ellipse.pose.position.x = center.x;
  ellipse.pose.position.y = center.y;
  return ellipse;
}

void DynamicPlanning::visualizeEllipse(obj_msgs::Obj obj)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = 50000 + ellipse_markers_.markers.size();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.2);

  tf::Quaternion quat;
  quat.setRPY(0, 0, obj.alpha);
  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  quat_msg.w = quat.w();
  marker.pose.orientation = quat_msg;
  marker.pose.position.x = obj.pose.position.x;
  marker.pose.position.y = obj.pose.position.y;
  marker.pose.position.z = 0.0;
  marker.scale.x = obj.a * 2.0;
  marker.scale.y = obj.b * 2.0;
  marker.scale.z = 0.1;

  marker.color.r = 0.75;
  marker.color.g = 0.75;
  marker.color.b = 0.75;
  marker.color.a = 0.8;
  ellipse_markers_.markers.push_back(marker);
}

void DynamicPlanning::mapInfoCallback(const path_msgs::Map map_msg)
{
  path_msgs::Map map_info = map_msg;

  for(int i = 0; i < map_info.links.size(); i++)
  {
    path_msgs::Link link = map_info.links[i];
    links_.insert(make_pair(link.id, link));
  }

  for(int i = 0; i < map_info.nodes.size(); i++)
  {
    path_msgs::Node node = map_info.nodes[i];
    nodes_.insert(make_pair(node.id, node));
  }

  for(int i = 0; i < map_info.lanes.size(); i++)
  {
    path_msgs::Lane lane = map_info.lanes[i];
    lanes_.insert(make_pair(lane.id, lane));
  }

  offsetX_ = map_info.OffsetMapX;
  offsetY_ = map_info.OffsetMapY;

  ROS_INFO_STREAM("Map information received");

  map_initialized_ = true;
}

void DynamicPlanning::currentOdomCallback(const nav_msgs::Odometry odom_msg)
{
  if (map_initialized_ && node_initialized_)
  {
    current_pose_.header.stamp = odom_msg.header.stamp;
    current_pose_.header.frame_id = map_frame_;
    current_pose_.pose = odom_msg.pose.pose;
    ego_.id = 0;
    ego_.type = OType::EGO;
    ego_.pose = odom_msg.pose.pose;
    // Transform from gps to hood or gps_to_rear
    yaw_ = tf::getYaw(ego_.pose.orientation);
    hood_pos_.x = ego_.pose.position.x + gps_to_hood_ * cos(yaw_);
    hood_pos_.y = ego_.pose.position.y + gps_to_hood_ * sin(yaw_);
    rear_pos_.x = ego_.pose.position.x - gps_to_rear_ * cos(yaw_);
    rear_pos_.y = ego_.pose.position.y - gps_to_rear_ * sin(yaw_);
    // visualizeCurrentPose();
    pose_initialized_ = true;
    startThread();
  }
}

void DynamicPlanning::trajectoryCallback(const path_msgs::Trajectory traj_msg)
{
  if (traj_msg.waypoints.size() > 0)
    trajectory_initialized_ = true;
  else
    trajectory_initialized_ = false;

  ROS_INFO_STREAM("Planned size: " << traj_msg.waypoints.size());

  global_trajectory_.waypoints.clear();
  for(int i = 0; i < traj_msg.waypoints.size(); i++)
  {
    if (i % 2 == 0)
    {
      global_trajectory_.waypoints.push_back(traj_msg.waypoints[i]);
    }
  }

  prev_index_ = -1;
  // Get bezier information
  // getBezierInfo();

  // Construct center line in s-p coordinate
  generateCenterLine();

  // Setup goal point
  path_msgs::Waypoint goal_wp = global_trajectory_.waypoints[global_trajectory_.waypoints.size()-1];
  goal_.pose.position.x = goal_wp.point.x;
  goal_.pose.position.y = goal_wp.point.y;
  goal_.velocity.x = 0.0;
  goal_.velocity.y = 0.0;
  goal_.type = OType::GOAL;
  goal_.is_wait = true;
  goal_.id = -(all_stop_nodes_.size() + 1);
  goal_.index = global_trajectory_.waypoints.size() - 1;
  goal_.s = global_len_;
  goal_.lat_offset = 0.0;
  goal_.link_id = goal_wp.CLID;
}


void DynamicPlanning::run()
{
  ros::Rate loop_rate(loop_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    ROS_INFO_STREAM("Current: x=" << ego_.pose.position.x <<", y=" << ego_.pose.position.y);

    if (pose_initialized_ && trajectory_initialized_ && map_initialized_)
    {
      ROS_WARN("Waiting for subscribing topics...");
      break;
    }
    ros::Duration(1.0).sleep();
  }

  bool reached = false;

  // relaying mode by default
  state_ = DynamicPlanning::STATE::RELAYING;

  // reset current index
  ego_.index = -1;

  // initialize stop nodes
  initializeStopNodes();

  while(ros::ok())
  {
    ros::spinOnce();
    rate_->sleep();
	}

  terminate_thread_ = true;
}

void DynamicPlanning::publishLocation()
{
  obj_msgs::Ego msg;
  msg.offset = ego_.lat_offset;
  path_msgs::Link clink = links_.at(global_trajectory_.waypoints[ego_.index].CLID);
  LControl status = laneStatus(clink);
  if (status == LControl::KEEP)
  {
    msg.is_rlink = false;
    msg.is_llink = false;
  }
  else if (status == LControl::RIGHT)
  {
    msg.is_rlink = true;
    msg.is_llink = false;
  }
  else if (status == LControl::LEFT)
  {
    msg.is_rlink = false;
    msg.is_llink = true;
  }
  else if (status == LControl::BOTH)
  {
    msg.is_rlink = true;
    msg.is_llink = false;
  }
  ego_location_pub_.publish(msg);
}

void DynamicPlanning::initializeStopNodes()
{
  obj_msgs::Obj node_155M01300104;
  obj_msgs::Obj node_155M00450204;
  obj_msgs::Obj node_155M01280104;
  obj_msgs::Obj node_155M01000104;
  obj_msgs::Obj node_155M01060104;
  obj_msgs::Obj node_155M01750104;
  obj_msgs::Obj node_155M01810104;
  obj_msgs::Obj node_155M00410104;
  // HD map 없는 곳 추가 필요
  // std::vector<obj_msgs::Obj> all_stop_nodes;

  // 왼쪽 비정형 출구
  node_155M01300104.type = OType::STOP;
  node_155M01300104.id = -1;
  node_155M01300104.pose.position.x = 471226.96 - offsetX_;
  node_155M01300104.pose.position.y = 3965790.51 - offsetY_;
  all_stop_nodes_.push_back(node_155M01300104);

  node_155M00450204.type = OType::STOP;
  node_155M00450204.id = -2;
  node_155M00450204.pose.position.x = 471221.4751 - offsetX_;
  node_155M00450204.pose.position.y = 3965780.1866 - offsetY_;
  all_stop_nodes_.push_back(node_155M00450204);

  node_155M00410104.type = OType::STOP;
  node_155M00410104.id = -8;
  node_155M00410104.pose.position.x = 471218.56 - offsetX_;
  node_155M00410104.pose.position.y = 3965797.92 - offsetY_;
  all_stop_nodes_.push_back(node_155M00410104);

  // 왼쪽 비정형 내 교차로
  node_155M01280104.type = OType::STOP;
  node_155M01280104.id = -3;
  node_155M01280104.pose.position.x = 471352.713 - offsetX_;
  node_155M01280104.pose.position.y = 3965763.814 - offsetY_;
  all_stop_nodes_.push_back(node_155M01280104);

  node_155M01000104.type = OType::STOP;
  node_155M01000104.id = -4;
  node_155M01000104.pose.position.x = 471362.959 - offsetX_;
  node_155M01000104.pose.position.y = 3965757.542 - offsetY_;
  all_stop_nodes_.push_back(node_155M01000104);

  node_155M01060104.type = OType::STOP;
  node_155M01060104.id = -5;
  node_155M01060104.pose.position.x = 471362.023- offsetX_;
  node_155M01060104.pose.position.y = 3965769.918 - offsetY_;
  all_stop_nodes_.push_back(node_155M01060104);

  // 하단 비정형
  node_155M01750104.type = OType::STOP;
  node_155M01750104.id = -6;
  node_155M01750104.pose.position.x = 471769.703 - offsetX_;
  node_155M01750104.pose.position.y = 3965545.995 - offsetY_;
  all_stop_nodes_.push_back(node_155M01750104);

  node_155M01810104.type = OType::STOP;
  node_155M01810104.id = -7;
  node_155M01810104.pose.position.x = 471775.441 - offsetX_;
  node_155M01810104.pose.position.y = 3965553.293 - offsetY_;
  all_stop_nodes_.push_back(node_155M01810104);

  // Reset
  for(int i = 0; i < all_stop_nodes_.size(); i++)
  {
    is_nodes_.push_back(true);
    timer_.push_back(0.0);
  }
}

void DynamicPlanning::generateStopNodes()
{
  bool is_vehicle_around = isVehicleAround();
  if (is_vehicle_around)
  {
    printf("Generating stop nodes....................\n");
    ROS_WARN("Generating stop nodes..................");
    for(int i = 0; i < all_stop_nodes_.size(); i++)
    {
      printf("node: %d --> x: %f, y: %f\n", i, all_stop_nodes_[i].pose.position.x, all_stop_nodes_[i].pose.position.y);
      is_nodes_.push_back(true);
      timer_.push_back(0.0);
    }
    stop_nodes_.clear();
    // Consider only valid stop nodes
    // printf("stop node size: %d\n", all_stop_nodes_.size());
    for(int i = 0; i < all_stop_nodes_.size(); i++)
    {
      obj_msgs::Obj node = all_stop_nodes_[i];
      // printf("Locating node...\n");
      locateObj(node);
      // printf("Node %d: index: %d, link_id: %s, arc: %f\n", i, node.index, node.link_id.c_str(), node.s);
      if (node.index > ego_.index && node.link_id != "")
      {
        node.is_wait = true;
        // 현재 차량이 Stop node에 가까이 있을
        if (getDistance(node.pose.position.x, node.pose.position.y, ego_.pose.position.x, ego_.pose.position.y) <= 15.0)
        {
          // 차량이 정차해있음
          if (ego_speed_ == 0.0)
          {
            double curr_time = ros::Time::now().toSec();
            // 첫 발견 시
            if (is_nodes_[i])
            {
              // printf("%d: First\n", i);
              timer_[i] = 0.0;
              is_nodes_[i] = false;
            }
            // 두 번째 이상 발견 시
            else
            {
              // printf("%d: More than first\n", i);
              // 이전 노드와 같은 노드라면 timer+
              if (prev_node_idx_ == i)
              {
                // printf("%d: Same node\n", i);
                timer_[i] += curr_time - prev_time_;
                ROS_WARN("Waiting for node: %f s", timer_[i]);
                printf("Waiting for node: %f s\n", timer_[i]);
              }
              // 이전 노드와 다르다면 timer reset
              else
              {
                // printf("%d: Different node\n", i);
                is_nodes_[i] = true;
                timer_[i] = 0.0;
              }
            }
            prev_time_ = curr_time;
            prev_node_idx_ = i;

            // printf("%d: Wait time: %f\n", timer_[i], i);

            // 차량이 설정한 값보다 오래 멈춰 있었으면
            if (timer_[i] > wait_time_thres_)
            {
              is_nodes_[i] = false;
              continue;
            }
            // 아직 좀 더 오래 멈춰있어야 한다면
            else
            {
              // is_nodes_[i] = true;
              stop_nodes_.push_back(node);
            }
          }
          // 가까이 있지만 정차하지 않았음
          else
          {
            // 만약 최근에 timer > wait_time_thres로 인해 지워지지 않은 노드라면
            if (is_nodes_[i])
            {
              stop_nodes_.push_back(node);
            }
          }
        }
        // 현재 차량이 Stop node에 가까이 있지 않을 때
        else
        {
          // printf("%d: Far\n", i);
          is_nodes_[i] = true;
          timer_[i] = 0.0;
          stop_nodes_.push_back(node);
        }
      }
      // 나의 global path 앞에 없을 때는 무조건 node 생성
      // else if (node.index <= ego_.index && node.s < ego_.s - (locate_range_))
      else if (getDistance(node.pose.position.x, node.pose.position.y, ego_.pose.position.x, ego_.pose.position.y) > 15.0)
      {
        is_nodes_[i] = true;
        timer_[i] = 0.0;
        stop_nodes_.push_back(node);
      }
    }
    visualizeStopNodes();
  }
}

void DynamicPlanning::locateEgo()
{
  geometry_msgs::Point point;
  point.x = ego_.pose.position.x;
  point.y = ego_.pose.position.y;

  int min_idx;
  if (prev_index_ == -1)
  {
    min_idx = 0;
  }
  else
  {
    min_idx = prev_index_;
  }

  double min_dist = getDistance(point.x, point.y, global_trajectory_.waypoints[min_idx].point.x, global_trajectory_.waypoints[min_idx].point.y);
  printf("Min idx: %d\n", min_idx);
  // Initial search
  if (min_idx == 0)
  {
    for(int i = 0; i < global_trajectory_.waypoints.size(); i++)
    {
      double dist = getDistance(point.x, point.y, global_trajectory_.waypoints[i].point.x, global_trajectory_.waypoints[i].point.y);
      if (dist < min_dist){
        min_dist = dist;
        min_idx = i;
      }
    }
  }
  // After initial search, search locally
  else if (min_idx > 0)
  {
    for (int i = min_idx; i < min_idx + 40; i++)
    {
      if (i < global_trajectory_.waypoints.size())
      {
        double dist = getDistance(point.x, point.y, global_trajectory_.waypoints[i].point.x, global_trajectory_.waypoints[i].point.y);
        if (dist < min_dist){
          min_dist = dist;
          min_idx = i;
        }
      }
    }
  }

  if (min_dist < locate_range_)
  {
    printf("Searched at %d\n", min_idx);
    ego_.index = min_idx;
    prev_index_ = ego_.index;
    ego_.s = std::get<0>(getSP(ego_));
    ego_.lat_offset = std::get<1>(getSP(ego_));
    ego_.link_id = getLinkID(ego_);
    publishLocation();
  }
  else
  {
    printf("Fail to search at %d\n", prev_index_);
    ROS_WARN("Fail to search at %d\n", prev_index_);
    ego_.index = prev_index_;
  }
  // error if outside the range
}

void DynamicPlanning::locateObj(obj_msgs::Obj &obj)
{
  if (ego_.index >= 0)
  {
    if (obj.type == OType::STOP)
    {
      obj.index = searchTrajIndex(obj, global_trajectory_, 1.5);
      if (obj.index >= 0)
      {
        obj.s = std::get<0>(getSP(obj)) + stop_push_;
        obj.lat_offset = std::get<1>(getSP(obj));
        obj.link_id = getLinkID(obj);
      }
    }
    else if (obj.type == OType::RED)
    {
      obj.index = searchTrajIndex(obj, global_trajectory_, 1.5);
      // printf("traffic index: %d\n", obj.index);
      if (obj.index >= 0)
      {
        obj.s = std::get<0>(getSP(obj)) + red_push_;
        obj.lat_offset = std::get<1>(getSP(obj));
        obj.link_id = getLinkID(obj);
      }
    }
    else if (obj.type == OType::ATYPICAL)
    {
      obj.index = searchTrajIndex(obj, global_trajectory_, locate_range_);
      if (obj.index >= 0)
      {
        obj.s = std::get<0>(getSP(obj));
        obj.lat_offset = std::get<1>(getSP(obj));
      }
    }
    else
    {
      obj.index = searchTrajIndex(obj, global_trajectory_, locate_range_);
      if (obj.index >= 0)
      {
        obj.s = std::get<0>(getSP(obj));
        obj.lat_offset = std::get<1>(getSP(obj));
        obj.link_id = getLinkID(obj);
      }
    }
  }
  else
  {
    obj.index = -1;
  }
  printf("locate obj: %d, %d, %d, %s\n", obj.type, obj.id, obj.index, obj.link_id.c_str());
}

void DynamicPlanning::checkAvoidance()
{
  // True: no avoidance
  // False: yes avoidance
  for(int i = 0; i < veh_list_.size(); i++)
  {
    obj_msgs::Obj obj = veh_list_[i];
    if (abs(obj.lat_offset) <= HALF_LANE)
    {
      bool is_veh = isVehicle(obj);
      if (is_veh)
      {
        printf("Obj %d: index: %d length: %f", obj.id, obj.index, obj.s);
        path_msgs::Link clink = links_.at(obj.link_id);
        std::string nlink_id = searchNextLink(obj.index);
        printf("Next link_id: %s\n", nlink_id.c_str());
        std::tuple<bool, bool, bool> is_before = isBeforeLTG(obj, std::make_tuple(light_thres_, turn_thres_, goal_thres_));

        // Object is an atypical object
        if(obj.type == OType::ATYPICAL)
        {
          obj.is_wait = false;
        }
        // Object at turn
        if (links_.at(obj.link_id).road_type == RType::TURN)
        {
          ROS_WARN("Object at turn --> WAIT");
          printf("Object at turn --> WAIT\n");
          obj.is_wait = true;
        }
        // Object at intersection (?)
        if (links_.at(obj.link_id).road_type == RType::INTERSECTION)
        {
          ROS_WARN("Object at intersection --> WAIT");
          printf("Object at intersection --> WAIT\n");
          obj.is_wait = true;
        }
        // Object is located where the current link KEEP
        if (laneStatus(clink) == LControl::KEEP)
        {
          ROS_WARN("No adjacent link --> WAIT");
          printf("No adjacent link --> WAIT\n");
          obj.is_wait = true;
        }
        // Ego at the current link KEEP
        if (laneStatus(links_.at(ego_.link_id)) == LControl::KEEP)
        {
          ROS_WARN("No adjacent link --> WAIT");
          printf("No adjacent link --> WAIT\n");
          obj.is_wait = true;
        }
        // Next link does not have lanes to change and object is close to the next link
        if (!nlink_id.empty())
        {
          path_msgs::Link nlink = links_.at(nlink_id);
          if (laneStatus(nlink) == LControl::KEEP)
          {
            int index = searchLinkIndex(obj.pose.position, clink);
            double distance_to_end = calcLength(clink, index, clink.geometry.size()-1);
            if (distance_to_end < light_thres_)
            {
              ROS_WARN("No adjacent link for the next link & too close to the end (%f m) --> WAIT", distance_to_end);
              printf("No adjacent link for the next link & too close to the end (%f m)--> WAIT\n", distance_to_end);
              obj.is_wait = true;
            }
            else
            {
              ROS_WARN("No adjacent link for the next link & far from the end (%f m) --> AVOID", distance_to_end);
              printf("No adjacent link for the next link & far from the end (%f m)--> AVOID\n", distance_to_end);
              obj.is_wait = false;
            }
          }
          else
          {
            printf("There are left or right to the next link --> AVOID\n");
            ROS_WARN("There are left or right to the next link --> AVOID");
            obj.is_wait = false;
          }
        }
        if (nlink_id.empty())
        {
          printf("There's no next link --> WAIT\n");
          ROS_WARN("There's no next link --> WAIT");
          obj.is_wait = true;
        }
        // Object is located before turn
        if (std::get<0>(is_before) || std::get<1>(is_before) || std::get<2>(is_before))
        {
          ROS_WARN("Before LTG --> WAIT");
          printf("Before LTG --> WAIT\n");
          obj.is_wait = true;
        }
        veh_list_[i] = obj;
      }
    }
  }
}

void DynamicPlanning::startThread()
{
  // while (!terminate_thread_)
  // {
  clock_t start = clock();
  if (trajectory_initialized_ && pose_initialized_)
  {
    ROS_WARN("*");
    // locateObj(ego_);
    locateEgo();
    generateStopNodes();
    printf("Ego: %s, index %d, arc %f, lat offset %f m\n", ego_.link_id.c_str(), ego_.index, ego_.s, ego_.lat_offset);
    printf("Goal: %s, index %d, arc %f, lat_offset %f m\n", goal_.link_id.c_str(), goal_.index, goal_.s, goal_.lat_offset);
    if (ego_.index >= 0 && ego_.link_id != "")
    {
      is_deadend_ = isDeadEnd();
      checkAvoidance();
      // Preliminary
      publishRoadState();
      calcRelativeYaw();
      printObjStatus();
      if (state_ == DynamicPlanning::STATE::RELAYING)
      {
        printf("=====Generating Dynamic Path ======\n");
        ROS_WARN("=====Generating Dynamic Path ======");
        generateDynamicPath();
      }
      if (state_ == DynamicPlanning::STATE::CRUISING)
      {
        printf("=====Executing Adaptive Cruise Control======\n");
        ROS_WARN("=====Executing Adaptive Cruise Control======");
        executeAdaptiveCruiseControl(headway_obj_);
      }
      else if (state_ == DynamicPlanning::STATE::BLOCKED)
      {
        printf("=====Following global path===================\n");
        ROS_WARN("=====Following global path===================");
        switchToGlobal(headway_obj_);
      }
      else if (state_ == DynamicPlanning::STATE::HARD_BRAKE)
      {
        printf("=======Hard brake =========\n");
        ROS_WARN("=======Hard brake =========");
        switchToGlobal(headway_obj_);
      }
      // Search object ahead
      searchObjAhead();
      updateObjAhead();
      switchState();
    }
    // dt_ = (double) (clock()-start)/CLOCKS_PER_SEC;
  }
  ROS_WARN("Execution time: %0.3f s", (float)(clock() - start)/CLOCKS_PER_SEC);
// }
}

void DynamicPlanning::hardBrake()
{
  // Publish maximum deceleration
  printf("Hard brake\n");
  std_msgs::Float64 msg;
  msg.data = -2.9;
  ego_acc_pub_.publish(msg);
}

void DynamicPlanning::switchToGlobal(obj_msgs::Obj obj)
{
  // Publish global path as reference path
  std::vector<geometry_msgs::Point> ref_path;
  for(int i = ego_.index; i < global_trajectory_.waypoints.size(); ++i)
  {
    if (i > obj.index)
    {
      break;
    }
    ref_path.push_back(global_trajectory_.waypoints[i].point);
  }
  publishReferencePath(ref_path);
  hardBrake();
}

void DynamicPlanning::publishRoadState()
{
  path_msgs::Link clink = links_.at(ego_.link_id);
  path_msgs::State msg;
  msg.state = clink.road_type;
  road_state_pub_.publish(msg);
}

void DynamicPlanning::calcRelativeYaw()
{
  double path_angle;
  if (ego_.index < global_trajectory_.waypoints.size()-1)
  {
    geometry_msgs::Point current_wp = global_trajectory_.waypoints[ego_.index].point;
    geometry_msgs::Point next_wp = global_trajectory_.waypoints[ego_.index+1].point;
    path_angle = atan2(next_wp.y-current_wp.y, next_wp.x-current_wp.x);
  }
  else
  {
    geometry_msgs::Point current_wp = global_trajectory_.waypoints[ego_.index].point;
    geometry_msgs::Point prev_wp = global_trajectory_.waypoints[ego_.index-1].point;
    path_angle = atan2(current_wp.y-prev_wp.y, current_wp.x-prev_wp.x);
  }
  relative_yaw_ = getAngleDiff(yaw_, path_angle);
}

int DynamicPlanning::getLateralDir(double cx, double cy, double sk)
{
  double x1 = sx_(sk);
  double y1 = sy_(sk);
  double x2 = sx_(sk + 0.5);
  double y2 = sy_(sk + 0.5);
  double delta_x = x2 - x1;
  double delta_y = y2 - y1;
  double slope = 0.0;
  if (delta_x != 0) {
    slope = delta_y / delta_x;
  }
  else {
    std::cout << "invalid" << std::endl;
    return 1;
  }

  double check = cy - y1 - slope * (cx - x1);

  // First and fourth quadrants
  if ((delta_y > 0 && delta_x > 0) || (delta_y < 0 && delta_x > 0))
  {
    if (check <= 0)
    {
      return -1;
    }
    else
    {
      return 1;
    }
  }
  // Second and third quadrants
  else if ((delta_y > 0 && delta_x < 0) || (delta_y < 0 && delta_x < 0))
  {
    if (check <= 0)
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
}

void DynamicPlanning::setOffsetCost()
{
  if(path_following_initialized_)
  {
    double sum = 0.0;
    std::vector<double> offsets(candidates_.size(), 0.0);
    // printf("ss %f, %f, %f\n", s_start_, s_end_, s_start_ + 8.0);
    for(int i = 0; i < candidates_.size(); i++)
    {
      // double lah_x = std::get<0>(candidates_[i].lah_point);
      // double lah_y = std::get<1>(candidates_[i].lah_point);
      double lah_x = candidates_[i].lah_point.x;
      double lah_y = candidates_[i].lah_point.y;
      double center_x = sx_(s_start_ + lookahead_distance_);
      double center_y = sy_(s_start_ + lookahead_distance_);
      double dist = getDistance(lah_x, lah_y, center_x, center_y);
      // printf("cand: %d, lah_x: %f, lah_y: %f, center_x: %f, center_y: %f\n", i, lah_x, lah_y, center_x, center_y);
      // double p = candidates_[i].sp(s_start_ + lookahead_distance_);
      // printf("%d candidate -- p: %f, LAH: %f\n", i, p, dist);
      // sum += p * p;
      // offsets[i] = p * p;
      sum += dist * dist;
      offsets[i] = dist * dist;
    }
    // Normalize
    for(int i = 0; i < offsets.size(); i++)
    {
      candidates_[i].gc = offsets[i] / sum;
    }
  }
}

// Consider continuity of a path for comfortability
void DynamicPlanning::setContinuityCost()
{
  // Continuity between candidates and previous optiaml path
  for(int i = 0; i < candidates_.size(); i++)
  {
    Candidate curr_cand = candidates_[i];
    if (prev_cand_.points.size() == 0)
    {
      candidates_[i].coc = 0.0;
    }
    else
    {
      double s1 = curr_cand.sp.m_x[0];
      double s2 = prev_cand_.sp.m_x[1];
      // printf("s1: %f, s2: %f\n", s1, s2);
      double sum = 0.0;
      for(double s0 = s1; (s0+0.2) <= s2; s0 += 0.2)
      {
        double prev_theta = atan2(prev_cand_.sp(s0+0.2) - prev_cand_.sp(s0), 0.2);
        double curr_theta = atan2(curr_cand.sp(s0+0.2) - curr_cand.sp(s0), 0.2);
        double delta_theta = abs(prev_theta - curr_theta);
        sum += delta_theta;
        // printf("prev_theta: %f, curr_theta: %f, delta %f\n", prev_theta, curr_theta, delta_theta);
      }
      candidates_[i].coc = 1.0 / (s2-s1) * sum;
      // printf("Path %d -- sum: %f, coc: %f\n", i, sum, candidates_[i].coc);
    }
  }
}

void DynamicPlanning::setR(int idx, bool is_obs, std::vector<path_msgs::Lane> contact_lanes)
{
  // Set obstacle R value
  if (is_obs)
  {
    R_all_[idx] = 1.0;
  }
  else
  {
    // Set lane R value
    for(int l = 0; l < contact_lanes.size(); l++)
    {
      // printf("Cand %d, t_lanes:%s\n", j, t_lanes[l].id.c_str());
      int lane_code = contact_lanes[l].lane_code;
      int lane_type = contact_lanes[l].lane_type;
      int line_type = lane_type % 10;
      // 실선
      // if (line_type == 1)
      // {
      //   R[j] = 1.0;
      // }

      // 주차금지선
      if (lane_code == LCode::UTURN || lane_code == LCode::NO_PARKING || lane_code == LCode::NO_STOPPING)
      {
        R_all_[idx] = 0.5;
        R_lanes_[idx] = 0.5;
      }
      else if (lane_code == LCode::CENTER)
      {
        R_all_[idx] = 0.5;
        R_lanes_[idx] = 0.5;
      }
      // 시설물 or 차선변경금지선
      else if (lane_code == LCode::BARRIER)
      {
        R_all_[idx] = 0.5;
        R_lanes_[idx] = 0.5;
      }
      // else if (lane_code == 99 || lane_code == 5)
      // {
      //   // printf("999 5555!\n");
      //   R[j] = 1.0;
      // }
      // ACC일 경우 차선도 장애물로 판단
      else if (lane_code == LCode::LANE)
      {
        // if (state_ == DynamicPlanning::STATE::CRUISING)
        // {
        //   // printf("333333!\n");
        //   R[j] = 1.0;
        // }
      }
      // else if (line_type == 1)
      // {
      //   R_[idx] = 0.5;
      // }
    }
  }
  // for(int i = 0; i < R_all_.size(); i++)
  // {
  //   printf("Cand: %d: R: %f\n", idx, R_all_[idx]);
  // }
}

bool DynamicPlanning::isBlocked()
{
  // Check whether all candidates are blocked
  int count = 0;
  for(int n = 0; n < R_all_.size(); n++)
  {
    if (R_all_[n] >= 0.5)
    {
      count++;
    }
  }
  if (count == R_all_.size())
  {
    return true;
  }
  else
  {
    return false;
  }
}

void DynamicPlanning::setObsCost()
{

  // Get adjacent lanes
  std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>> adj_lanes = getAdjacentLanes(ego_.link_id);
  std::vector<path_msgs::Lane> llanes = std::get<0>(adj_lanes);
  std::vector<path_msgs::Lane> rlanes = std::get<1>(adj_lanes);
  std::vector<path_msgs::Lane> all_adj_lanes = llanes;
  all_adj_lanes.insert(all_adj_lanes.end(), rlanes.begin(), rlanes.end());
  printf("Adj lanes size: %d\n", all_adj_lanes.size());
  for(int i = 0; i < all_adj_lanes.size(); i++)
  {
    printf("%s\n", all_adj_lanes[i].id.c_str());
  }
  // reset R's
  std::vector<double> R(2*path_size_+1, 0.0);
  R_all_ = R;
  R_lanes_ = R;

  for (int j = 0; j < candidates_.size(); j++)
  {
    // printf("--------Candidate %d---------\n", j);
    checkPath(candidates_[j].points, all_adj_lanes, j);
  }

  // Gaussian convolution for R_all_
  for(int i = 0; i < candidates_.size(); i++)
  {
    double oc = 0.0;
    for(int k = -path_size_; k <= path_size_; k++)
    {
      double Rki;
      double g = 1.0 / sqrt(2*PI*sigma_*5) * exp(-pow(k,2) / (2*pow(sigma_*5, 2)));
      if (k+i < 0 || k+i > 2*path_size_)
      {
        Rki = 0.5;
      }
      else
      {
        Rki = R_all_[k+i];
      }
      oc += g * Rki;
    }
    candidates_[i].oc = oc;
  }

  // Gaussian convolution for R_lanes_
  for(int i = 0; i < candidates_.size(); i++)
  {
    double lc = 0.0;
    for(int k = -path_size_; k <= path_size_; k++)
    {
      double Rki;
      double g = 1.0 / sqrt(2*PI*sigma_) * exp(-pow(k,2) / (2*pow(sigma_, 2)));
      if (k+i < 0 || k+i > 2*path_size_)
      {
        Rki = 0.5;
      }
      else
      {
        Rki = R_lanes_[k+i];
      }
      lc += g * Rki;
    }
    candidates_[i].lc = lc;
  }
}

// Find adjacent lanes
std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>>
DynamicPlanning::getAdjacentLanes(std::string link_id)
{
  path_msgs::Link clink = links_.at(link_id);
  std::vector<path_msgs::Lane> rlanes;
  std::vector<path_msgs::Lane> llanes;
  std::deque<path_msgs::Link> links;
  std::deque<bool> state;
  links.push_back(clink);
  state.push_back(true);

  while(!(links.size() == 0))
  {
    path_msgs::Link link = links.front();
    bool is_search = state.front();
    links.pop_front();
    state.pop_front();

    LControl lane_status = laneStatus(link);
    // printf("Find current link's lanes\n");
    // Find selected link's lanes
    for(int i = 0; i < link.R_laneID.size(); i++)
    {
      // printf("link's right lane: %s\n", link.R_laneID[i].c_str());
      path_msgs::Lane rlane = lanes_.at(link.R_laneID[i]);
      if(!isSafeLane(rlane))
      {
        rlanes.push_back(rlane);
      }
    }
    for(int i = 0; i < link.L_laneID.size(); i++)
    {
      // printf("link's left lane: %s\n", link.L_laneID[i].c_str());
      path_msgs::Lane llane = lanes_.at(link.L_laneID[i]);
      if(!isSafeLane(llane))
      {
        llanes.push_back(llane);
      }
    }

    // Find selected link's right lane
    if (lane_status == LControl::RIGHT)
    {
      // printf("Add right link's next links' lanes\n");
      path_msgs::Link rlink = links_.at(link.RLID[0]);
      for(int i = 0; i < rlink.R_laneID.size(); i++)
      {
        path_msgs::Lane rlane = lanes_.at(rlink.R_laneID[i]);
        if(!isSafeLane(rlane))
        {
          rlanes.push_back(rlane);
        }
      }
    }
    // Find selected link's left lane
    else if (lane_status == LControl::LEFT)
    {
      // printf("Add left link's next links' lanes\n");
      path_msgs::Link llink = links_.at(link.LLID[0]);
      for(int i = 0; i < llink.L_laneID.size(); i++)
      {
        path_msgs::Lane llane = lanes_.at(llink.L_laneID[i]);
        if(!isSafeLane(llane))
        {
          llanes.push_back(llane);
        }
      }
    }
    // Find selected link's both lane
    else if (lane_status == LControl::BOTH)
    {
      // printf("Add left and right link's next links' lanes\n");
      path_msgs::Link rlink = links_.at(link.RLID[0]);
      path_msgs::Link llink = links_.at(link.LLID[0]);
      for(int i = 0; i < rlink.R_laneID.size(); i++)
      {
        path_msgs::Lane rlane = lanes_.at(rlink.R_laneID[i]);
        if(!isSafeLane(rlane))
        {
          rlanes.push_back(rlane);
        }
      }
      for(int i = 0; i < llink.L_laneID.size(); i++)
      {
        path_msgs::Lane llane = lanes_.at(llink.L_laneID[i]);
        if(!isSafeLane(llane))
        {
          llanes.push_back(llane);
        }
      }
    }

    // Search more next link's lanes
    if (is_search)
    {
      for(int j = 0; j < link.NLIDS.size(); j++)
      {
        if (!link.NLIDS[j].empty())
        {
          // printf("next link id: %s\n", link.NLIDS[j].c_str());
          path_msgs::Link nlink = links_.at(link.NLIDS[j]);
          links.push_back(nlink);
          // printf("nlink legnth: %f\n", nlink.length);
          if (nlink.length > safety_distance_)
          {
            state.push_back(false);
          }
          else
          {
            state.push_back(true);
          }
        }
      }
    }
  }

  // DEBUG
  for(int i = 0; i < rlanes.size(); i++)
  {
    // printf("RLANES %s\n", rlanes[i].id.c_str());
  }
  for(int i = 0; i < llanes.size(); i++)
  {
    // printf("LLANES %s\n", llanes[i].id.c_str());
  }
  return std::make_tuple(llanes, rlanes);
}


bool DynamicPlanning::isReached()
{
  if (distanceToIndex(sx_, ego_.index, stop_dist_) == sx_.m_x.size()-1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool DynamicPlanning::isCollision(double x, double y, obj_msgs::Obj obj) {

  if (is_rect_)
  {
    // AB = Upper-left -> Upper-right
    // AD = Upper-left -> Bottom-left
    // AM = Upper-left -> target point
    std::vector<double> AB {obj.ur.x - obj.ul.x, obj.ur.y - obj.ul.y};
    std::vector<double> AD {obj.bl.x - obj.ul.x, obj.bl.y - obj.ul.y};
    std::vector<double> AM {x - obj.ul.x, y - obj.ul.y};
    double dot_AB_AM = std::inner_product(AM.begin(), AM.end(), AB.begin(), 0);
    double dot_AB_AB = std::inner_product(AB.begin(), AB.end(), AB.begin(), 0);
    double dot_AM_AD = std::inner_product(AM.begin(), AM.end(), AD.begin(), 0);
    double dot_AD_AD = std::inner_product(AD.begin(), AD.end(), AD.begin(), 0);
    if(dot_AB_AM >= 0 && dot_AB_AM <= dot_AB_AB && dot_AM_AD >= 0 && dot_AM_AD <= dot_AD_AD)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  /* Ellipse */
  else if (is_ellipse_)
  {
    double angle = obj.alpha;
    double a = obj.a;
    double b = obj.b;
    double x0 = obj.pose.position.x;
    double y0 = obj.pose.position.y;
    double equation = ((x-x0)*cos(angle) + (y-y0)*sin(angle))*((x-x0)*cos(angle) + (y-y0)*sin(angle)) / (a*a) +
                        ((x-x0)*sin(angle) - (y-y0)*cos(angle))*((x-x0)*sin(angle) - (y-y0)*cos(angle)) / (b*b);
    return (equation <= 1) ? true : false;
  }

}

void DynamicPlanning::transformObject(const obj_msgs::Obj src, obj_msgs::Obj &dest)
{
  // Find transform
  tf::Transform base_link2map;
  base_link2map.setRotation(tf::Quaternion(ego_.pose.orientation.x,
                                      ego_.pose.orientation.y,
                                      ego_.pose.orientation.z,
                                      ego_.pose.orientation.w));
  base_link2map.setOrigin(tf::Vector3(ego_.pose.position.x,
                                  ego_.pose.position.y,
                                  ego_.pose.position.z));

  // Transform object from local to global frame
  tf::Vector3 lpose;
  tf::Vector3 lvel;
  // tf::Vector3 gvel;
  lpose.setValue(src.pose.position.x, src.pose.position.y, 0.0);

  // Local upper-left, upper-right, bottom-left, bottom-right
  tf::Vector3 lul; tf::Vector3 lur; tf::Vector3 lbl; tf::Vector3 lbr;
  lul.setValue(src.ul.x, src.ul.y, 0.0);
  lur.setValue(src.ur.x, src.ur.y, 0.0);
  lbl.setValue(src.bl.x, src.bl.y, 0.0);
  lbr.setValue(src.br.x, src.br.y, 0.0);

  // Local velocity
  lvel.setValue(src.velocity.x, src.velocity.y, 0.0);

  tf::Vector3 gpose = base_link2map * lpose;
  tf::Vector3 gul = base_link2map * lul;
  tf::Vector3 gur = base_link2map * lur;
  tf::Vector3 gbl = base_link2map * lbl;
  tf::Vector3 gbr = base_link2map * lbr;

  // Velocity transform ignores translation
  base_link2map.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Vector3 gvel = base_link2map * lvel;

  // Transformed object
  dest.pose.position.x = gpose.x();
  dest.pose.position.y = gpose.y();
  dest.velocity.x = gvel.x();
  dest.velocity.y = gvel.y();
  dest.ul.x = gul.x();
  dest.ul.y = gul.y();
  dest.ur.x = gur.x();
  dest.ur.y = gur.y();
  dest.bl.x = gbl.x();
  dest.bl.y = gbl.y();
  dest.br.x = gbr.x();
  dest.br.y = gbr.y();
}

void DynamicPlanning::checkPath(std::vector<geometry_msgs::Point> points,
                                std::vector<path_msgs::Lane> adj_lanes,
                                int cand_idx)
{
  bool is_obs = false;
  obj_msgs::Obj reset;
  candidates_[cand_idx].collision_obj = reset;
  for(int i = 0; i < veh_list_.size(); i++)
  {
    obj_msgs::Obj obj = veh_list_[i];
    bool is_veh = isVehicle(obj);
    if (is_veh)
    {
      for (int j = 0; j < points.size(); j++)
      {
        double x = points[j].x;
        double y = points[j].y;
        // Approaching
        if (obj.type == OType::APPROACHING || obj.type == OType::FRONTOBJ)
        {
          // printf("Vel: %f %f\n", obj.velocity.x, obj.velocity.y);
          // printf("Approaching magnitude: %f\n", sqrt(obj.velocity.x*obj.velocity.x + obj.velocity.y*obj.velocity.y));
          for(double t = 0.0; t <= 10.0; t+= 0.1)
          {
            obj_msgs::Obj tracker = obj;
            if (is_rect_)
            {
              tracker.ul.x = obj.ul.x + t * obj.velocity.x;
              tracker.ul.y = obj.ul.y + t * obj.velocity.y;
              tracker.ur.x = obj.ur.x + t * obj.velocity.x;
              tracker.ur.y = obj.ur.y + t * obj.velocity.y;
              tracker.bl.x = obj.bl.x + t * obj.velocity.x;
              tracker.bl.y = obj.bl.y + t * obj.velocity.y;
            }
            else if (is_ellipse_)
            {
              tracker.alpha = obj.alpha;
              tracker.a = obj.a;
              tracker.b = obj.b;
              tracker.pose.position.x = obj.pose.position.x + t * obj.velocity.x;
              tracker.pose.position.y = obj.pose.position.y + t * obj.velocity.y;
            }
            if (isCollision(x, y, tracker))
            {
              candidates_[cand_idx].collision_point = points[j];
              candidates_[cand_idx].collision_obj = tracker;
              candidates_[cand_idx].time_to_collision = t;
              printf("Cand: %d, Approaching: collision..., index: %d, type: %d, id: %d\n", cand_idx, obj.index, obj.type, obj.id);
              printf("Collision point -> x: %f, y: %f, ttc: %f\n", points[j].x, points[j].y, t);
              is_obs = true;
              break;
            }
          }
          if (is_obs) break;
        }
        else
        {
          if (isCollision(x, y, obj))
          {
            is_obs = true;
            candidates_[cand_idx].collision_point = points[j];
            candidates_[cand_idx].collision_obj = obj;
            printf("Cand: %d, static obj id: %d, obj type: %d, detected\n", cand_idx, obj.id, obj.type);
            break;
          }
        }
      }
    }
  }

  // Check lane contact
  std::vector<path_msgs::Lane> contact_lanes;
  for(auto it = adj_lanes.begin(); it != adj_lanes.end(); it++)
  {
    // printf("Adj lanes: %s\n", it->id.c_str());
    if (!isSafeLane(*it))
    {
      for(int k = it->geometry.size()-1 ; k >=0; k--)
      {
        for(int j = 0; j < points.size(); j++)
        {
          double x = points[j].x;
          double y = points[j].y;
          double dist = getDistance(x, y, it->geometry[k].x - offsetX_, it->geometry[k].y - offsetY_);
          if (dist < lane_thres_)
          {
            if (std::find(contact_lanes.begin(), contact_lanes.end(), *it) == contact_lanes.end())
            {
              // obj_msgs::Obj lane;
              // lane.type = OType::LANES;
              // candidates_[cand_idx].collision_point = points[j];
              // candidates_[cand_idx].collision_obj = lane;
              printf("Cand: %d, contact lanes: %s\n",cand_idx, it->id.c_str());
              contact_lanes.push_back(*it);
            }
          }
        }
      }
    }
  }
  setR(cand_idx, is_obs, contact_lanes);
}

LControl DynamicPlanning::laneStatus(path_msgs::Link clink)
{
  if (clink.LLID[0].empty() && clink.RLID[0].empty())
  {
    // printf("Lane change not available\n");
    return LControl::KEEP;
  }
  else if(!clink.LLID[0].empty() && clink.RLID[0].empty())
  {
    // printf("Left lane available\n");
    return LControl::LEFT;
  }
  else if(!clink.RLID[0].empty() && clink.LLID[0].empty())
  {
    // printf("Right lane available\n");
    return LControl::RIGHT;
  }
  else
  {
    // printf("Both lanes are available\n");
    return LControl::BOTH;
  }
}

/* RVIZ visualization */
void DynamicPlanning::visualizeStopNodes()
{
  visualization_msgs::MarkerArray node_markers;
  for(int i=0; i < stop_nodes_.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.id = 30000 + i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = stop_nodes_[i].pose.position.x;
    marker.pose.position.y = stop_nodes_[i].pose.position.y;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 0.5;
    marker.color.r = 0.643;
    marker.color.g = 0.369;
    marker.color.b = 0.941;
    marker.color.a = 1.0;
    node_markers.markers.push_back(marker);
  }
  rviz_stop_nodes_pub_.publish(node_markers);
}

void DynamicPlanning::visualizeGoalPoint()
{
  visualization_msgs::Marker rviz_goal;
  rviz_goal.header.frame_id = map_frame_;
  rviz_goal.header.stamp = ros::Time::now();
  rviz_goal.action = visualization_msgs::Marker::ADD;
  rviz_goal.pose.orientation.w = 1.0;
  rviz_goal.id = 5000;
  rviz_goal.type = visualization_msgs::Marker::POINTS;
  rviz_goal.scale.x = 2;
  rviz_goal.scale.y = 2;
  rviz_goal.color.g = 1.0;
  rviz_goal.color.a = 1.0;
  rviz_goal.points.push_back(goal_.pose.position);
  rviz_goal_point_pub_.publish(rviz_goal);
}

void DynamicPlanning::visualizeCurrentPose()
{

  visualization_msgs::Marker rviz_pose;

  rviz_pose.header.frame_id = map_frame_;
  rviz_pose.header.stamp = ros::Time::now();
  rviz_pose.action = visualization_msgs::Marker::ADD;
  rviz_pose.pose = ego_.pose;
  rviz_pose.id = 0;
  rviz_pose.type = visualization_msgs::Marker::ARROW;
  rviz_pose.scale.x = 1.0;
  rviz_pose.scale.y = 1.0;
  rviz_pose.color.r = 1.0;
  rviz_pose.color.g = 0.0;
  rviz_pose.color.b = 0.0;
  rviz_pose.color.a = 1.0;

  rviz_current_pose_pub_.publish(rviz_pose);
}

std::tuple<double, double> DynamicPlanning::getSP(obj_msgs::Obj obj)
{
  double cx = obj.pose.position.x;
  double cy = obj.pose.position.y;
  int init_index = obj.index;
  // Quadratic Minimization method
  std::vector<double> ss(4);
  int iteration = 0;
  double sk_last;
  // Initial three
  double init_s = sx_.m_x[init_index];
  ss[0] = init_s - 7.0;
  ss[2] = init_s + 7.0;
  ss[1] = (ss[0] + ss[2]) / 2;
  // std::cout << "Getting lateral" << std::endl;
  while(true)
  {
    double s1 = ss[0];
    double s2 = ss[1];
    double s3 = ss[2];
    double Ds1 = getDistance(sx_(s1), sy_(s1), cx, cy);
    double Ds2 = getDistance(sx_(s2), sy_(s2), cx, cy);
    double Ds3 = getDistance(sx_(s3), sy_(s3), cx, cy);
    double sk = 0.5 * ((pow(s2,2)-pow(s3,2))*Ds1 + (pow(s3,2)-pow(s1,2))*Ds2 + (pow(s1,2)-pow(s2,2))*Ds3)
                / ((s2-s3)*Ds1 + (s3-s1)*Ds2 + (s1-s2)*Ds3);

    // std::cout << "s_init: " << init_s << std::endl;
    // std::cout << "s1: " << s1 << std::endl;
    // std::cout << "s2: " << s2 << std::endl;
    // std::cout << "s3: " << s3 << std::endl;
    // ROS_INFO("%f", (pow(s2,2)-pow(s3,2))*Ds1);
    // ROS_INFO("%f", (pow(s3,2)-pow(s1,2))*Ds2);
    // ROS_INFO("%f", (pow(s1,2)-pow(s2,2))*Ds3);
    // ROS_INFO("%f", (s2-s3)*Ds1);
    // ROS_INFO("%f", (s3-s1)*Ds2);
    // ROS_INFO("%f", (s1-s2)*Ds3 );
    // ROS_INFO("%f", sk);
    //
    // std::cout << "Ds1: " << Ds1 << std::endl;
    // std::cout << "Ds2: " << Ds2 << std::endl;
    // std::cout << "Ds3: " << Ds3 << std::endl;

    // // Newton method
    // double width = 1.0 / (sx_.m_x.size()*1000);
    double term_cond = 1.0 / (sx_.m_x.size()*1000);
    // sk = clamp(sk, init_s-2.0, init_s + 3.0);
    // double grad, curv;
    // double Ds_pt1 = getDistance(sx_(sk-width), sy_(sk-width), cx, cy);
    // double Ds_pt2 = getDistance(sx_(sk), sy_(sk), cx, cy);
    // double Ds_pt3 = getDistance(sx_(sk+width), sy_(sk+width), cx, cy);
    // double g1 = (Ds_pt2 - Ds_pt1) / width;
    // double g2 = (Ds_pt3 - Ds_pt2) / width;
    // grad = (Ds_pt3 - Ds_pt1) / (2*width);
    // curv = (g2 - g1) / width;
    //
    // std::cout << "g1: " << g1 << std::endl;
    // std::cout << "g2: " << g2 << std::endl;
    // std::cout << "grad: " << grad << std::endl;
    // std::cout << "curv: " << curv << std::endl;
    // if (curv != 0.0)
    // {
    //   sk = sk - grad / curv;
    //   sk = clamp(sk, init_s-2.0, init_s + 3.0);
    // }
    ss[3] = sk;

    // ROS_INFO("new sk: %f", sk);

    if(iteration > 0)
    {
      if (abs(sk_last-sk) < term_cond)
      {
        double lat_offset = getDistance(sx_(sk), sy_(sk), cx, cy);
        lat_offset = getLateralDir(cx, cy, sk) * lat_offset;
        // printf("Lateral offset: %f\n", lat_offset);
        return std::make_tuple(sk, lat_offset);
      }
    }

    sk_last = sk;


    // Eliminate the value which gives the largest P(s) among s1 s2 s3 sk
    double max_P = -1 * std::numeric_limits<double>::infinity();
    int max_idx = 0;

    for(int i = 0 ; i < 4; i++)
    {
      double s = ss[i];
      double P = (s-s2)*(s-s3)*Ds1 / ((s1-s2)*(s1-s3)) +
              (s-s1)*(s-s3)*Ds2 / ((s2-s1)*(s2-s3)) +
                  (s-s1)*(s-s2)*Ds3 / ((s3-s1)*(s3-s2));
      // std::cout << "P: " << P << std::endl;

      if (P > max_P)
      {
        max_P = P;
        max_idx = i;
      }
    }

    // std::cout << max_idx << std::endl;
    if (max_idx < 3)
    {
      ss[max_idx] = sk;
      for (int i = 0; i < 3; i++)
      {
        for (int j=i+1; j < 3; j++)
        {
          if (ss[i] == ss[j])
          {
            if (ss[j] < 0.5) ss[j] = ss[j] + 0.0001;
            else ss[j] = ss[j] - 0.0001;
          }
        }
      }
    }

    if(iteration > 100)
    {
      ROS_INFO_STREAM("Fail to find optimal lateral offset");
      double lat_offset = getDistance(sx_(init_s), sy_(init_s), cx, cy);
      // ROS_INFO("Lateral offset: %f", lat_offset);
      return std::make_tuple(init_s, lat_offset);
    }
    iteration++;
  }
}

void DynamicPlanning::generateCenterLine()
{
  double length = 0.0;
  std::vector<double> x; // lateral offset
  std::vector<double> y; // lateral offset
  std::vector<double> sl; // arc length

  x.push_back(global_trajectory_.waypoints[0].point.x);
  y.push_back(global_trajectory_.waypoints[0].point.y);
  sl.push_back(length);

  for(int i=1; i < global_trajectory_.waypoints.size(); i++)
  {
    geometry_msgs::Point wp = global_trajectory_.waypoints[i].point;
    geometry_msgs::Point prev_wp = global_trajectory_.waypoints[i-1].point;
    double step = sqrt(pow(prev_wp.x-wp.x, 2) + pow(prev_wp.y-wp.y,2));
    if (step > 0.0)
    {
      length += step;
      x.push_back(global_trajectory_.waypoints[i].point.x);
      y.push_back(global_trajectory_.waypoints[i].point.y);
      sl.push_back(length);
    }
  }
  global_len_ = length;
  sx_.set_points(sl,x);
  sy_.set_points(sl,y);
}

std::tuple<double, double> DynamicPlanning::locateObjectBackFromEgo(obj_msgs::Obj obj)
{
  obj_msgs::Obj fake = obj;
  fake.pose.position.x = (obj.bl.x + obj.br.x) / 2.0;
  fake.pose.position.y = (obj.bl.y + obj.br.y) / 2.0;
  locateObj(fake);
  if (fake.index >= 0 && fake.link_id != "")
  {
    return (std::make_tuple(fake.s - ego_.s, abs(fake.lat_offset - ego_.lat_offset)));
  }
  else
  {
    locateObjectFromEgo(obj);
  }
}

// Returns arc-length and lateral offset difference b/t an object and ego
// Need to consider the size of the object
std::tuple<double, double> DynamicPlanning::locateObjectFromEgo(obj_msgs::Obj obj)
{
  return (std::make_tuple(obj.s - ego_.s, abs(obj.lat_offset - ego_.lat_offset)));
}

double DynamicPlanning::getSafetyDistance()
{
  // Set safety distance according to current ego speed
  // 1.5 optimal
  double safety_distance = max_dist_ * (ego_speed_ / ref_speed_) / 1.5;
  path_msgs::Link clink = links_.at(ego_.link_id);
  int start_bezier = 0;
  bool is_bezier = false;
  double length = 0.0;

  if (clink.road_type == RType::ATYPICAL_TURN)
  {
    ROS_WARN("ROAD STATE --> Atypical Turn");
    // return 8.0;
  }
  else
  {
    for(int i=ego_.index; i < global_trajectory_.waypoints.size()-1; i++)
    {
      // printf("Next link: %s\n", searchNextLink(ego_.index).c_str());
      path_msgs::Waypoint curr_wp = global_trajectory_.waypoints[i];
      path_msgs::Waypoint next_wp = global_trajectory_.waypoints[i+1];
      if (curr_wp.CLID == searchNextLink(ego_.index))
      {
        if (links_.at(curr_wp.CLID).road_type == RType::ATYPICAL_TURN )
        {
          double distance = sx_.m_x[i] - sx_.m_x[ego_.index];
          if (distance <= 15.0)
          {
            path_msgs::State msg;
            msg.state = RType::BEFORE_TURN;
            road_state_pub_.publish(msg);
            ROS_WARN("ROAD STATE --> Before Turn");
            // return 8.0;
          }
        }
      }
    }
  }

  if (safety_distance < min_dist_)
  {
    safety_distance = min_dist_;
  }
  else if (safety_distance > max_dist_)
  {
    safety_distance = max_dist_;
  }

  // avoidance를 시작할 때 처음 계산된 물체와의 거리를 safety distance로 유지한다.
  // avoidance 대상이 움직이기 시작하면 opt_path_wo_obs_가 free해진다.
  bool is_veh = isVehicle(headway_obj_);
  if (look_further_ && is_veh)
  {
    if (!first_sight_)
    {
      avoidance_distance_ = std::get<0>(locateObjectBackFromEgo(headway_obj_));
      first_sight_ = true;
    }
    if (avoidance_distance_ > safety_distance)
    {
      return avoidance_distance_;
    }
    else
    {
      return safety_distance;
    }
  }
  else
  {
    first_sight_ = false;
  }
  return safety_distance;
}

int DynamicPlanning::distanceToIndex(tk::spline s, int idx, double dist)
{
  double s_start = s.m_x[idx];
  for(int i = idx; i < s.m_x.size(); i++)
  {
    // printf("%f %f %f\n", s.m_x[i], s_start, dist);
    if (s.m_x[i] > s_start + dist)
    {
      printf("Looking at index %d\n", i);
      return i;
    }
  }
  printf("No index to look for\n");
  return (s.m_x.size()-1);
}

void DynamicPlanning::generateCandidates(double min_offset, int safety_index)
{
  candidates_.clear();
  // int count_s = 7;
  // int count_m = 5;
  // int count_l = 3;
  // double inter_s = min_offset;
  // double inter_m = min_offset * 4;
  // double inter_l = min_offset * 8;
  int count_s = 35;
  int count_m = 0;
  int count_l = 0;
  double inter_s = min_offset;
  double inter_m = min_offset*2;
  double inter_l = min_offset*3;

  double coverage = count_s * inter_s + count_m * inter_m + count_l * inter_l;
  double remainder = coverage;
  for(int i = 0; i < 2*path_size_+1; i++)
  {
    if (i < count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, safety_index));
      remainder -= inter_l;
    }
    else if (i < count_l + count_m)
    {
      candidates_.push_back(generatePathCandidate(remainder, safety_index));
      remainder -= inter_m;
    }
    else if (i < 2*count_s + count_m + count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, safety_index));
      remainder -= inter_s;
    }
    else if (i < 2*(count_s + count_m) + count_l)
    {
      candidates_.push_back(generatePathCandidate(remainder, safety_index));
      remainder -= inter_m;
    }
    else if (i <= 2*(count_s + count_m + count_l))
    {
      candidates_.push_back(generatePathCandidate(remainder, safety_index));
      remainder -= inter_l;
    }
  }
}

void DynamicPlanning::findOptimalPath()
{
  double min_cost_w = 10000.0;
  double min_cost_wo = 10000.0;
  int min_index_w = 0;
  int min_index_wo = 0;
  // printf("smoothness, static, offset\n");
  for(int i=0; i<candidates_.size();i++)
  {
    double cost_w = wc_*(a_ * candidates_[i].smc + b_*candidates_[i].coc) + ws_*candidates_[i].oc + wl_*candidates_[i].gc;
    double cost_wo = wc_*(a_ * candidates_[i].smc + b_*candidates_[i].coc) + ws_*candidates_[i].lc + wl_*candidates_[i].gc;
    if (cost_w < min_cost_w)
    {
      min_index_w = i;
      min_cost_w = cost_w;
    }
    if (cost_wo < min_cost_wo)
    {
      min_index_wo = i;
      min_cost_wo = cost_wo;
    }
    // printf("%d cost: %3f, %3f, %3f %f --> %3f, %3f\n",
    // i,
    // wc_*(a_ * candidates_[i].smc + b_*candidates_[i].coc),
    // ws_*candidates_[i].oc,
    // wl_*candidates_[i].gc,
    // ws_*candidates_[i].lc, cost_wo, cost_w);
  }
  opt_path_ = min_index_w;
  opt_path_wo_obs_ = min_index_wo;
  printf("Optimal path: %d\n", opt_path_);
  printf("Optimal path w/o obstacles: %d\n", opt_path_wo_obs_);
}

void DynamicPlanning::setTargetSpeed(double target_speed)
{
  // Publish target speed
  std_msgs::Float64 msg;
  msg.data = target_speed;
  ego_speed_pub_.publish(msg);
}


void DynamicPlanning::switchRelaying()
{
  // find optimal path index w/o obstacle and w/ obstacle
  findOptimalPath();

  // find optimal path candidate w/o obstacle collides w/ obstacle
  Candidate cand = candidates_[opt_path_wo_obs_];

  // Check the collision object of the optimal candidate
  obj_msgs::Obj obj = cand.collision_obj;
  if (obj.type == OType::APPROACHING || obj.type == OType::FRONTOBJ)
  {
    // dtc: distance-to-point-of-collision
    double obj_speed = sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y*obj.velocity.y);
    double obj_dtpoc = obj_speed * cand.time_to_collision;
    double ego_speed_ms = ego_speed_ * 1000.0 / 3600.0;
    double ego_dtpoc = ego_speed_ms * cand.time_to_collision;
    printf("TTC: %f\n", cand.time_to_collision);
    ROS_WARN("obj's speed: %f, distance-to-point-of-collision: %f", obj_speed, obj_dtpoc);
    printf("obj's speed: %f, distance-to-point-of-collision: %f\n", obj_speed, obj_dtpoc);
    ROS_WARN("Ego's speed: %f, distance-to-point-of-collision: %f", ego_speed_ms, ego_dtpoc);
    printf("Ego's speed: %f, distance-to-point-of-collision: %f\n", ego_speed_ms, ego_dtpoc);

    // 만약 현재 차량 속도로 ttc 시간 만큼 주행 했을 경우의 거리에 safety distance를 더한 것이
    // object가 ttc 시간만큼 주행했을 때 거리보다 짧을 경우에만 ACC를 진행하도록 전방 물체를 만든다.
    // if (ego_dtpoc - obj_dtpoc <= safety_distance_)
    if (ego_dtpoc - obj_dtpoc <= cut_in_thres_)
    {
      is_brake_ = true;
    }
    else
    {
      printf("Not within the cut-in threshold\n");
      is_brake_ = false;
    }
    // approaching과 충돌의 경우 타이머 리셋 & avoidance 불가
    is_avoid_ = false;
    static_timer_ = 0.0;
  }
  // collision with static or front objects
  else if (obj.type == OType::STATIC)
  {
    printf("Collision with STATIC\n");
    double curr_time = ros::Time::now().toSec();
    // 10 seconds pass but do not avoid
    if (obj.is_wait)
    {
      printf("Do not avoid\n");
      is_brake_ = true;
      is_avoid_ = false;
      is_avoiding_ = false;
      static_timer_ = 0.0;
    }
    // 10 seconds pass then avoid
    else
    {
      // ACC에서 avoidance로 넘어온 경우 avoidance 시간 기다리지 않고 avoid
      if (look_further_)
      {
        printf("Avoidance after ACC...\n");
        is_brake_ = false;
        is_avoid_ = true;
        is_avoiding_ = true;
        static_timer_ = 0.0;
      }
      // ACC에서 avoidance로 넘어온 것이 아닌 경우 avoidance 시간 기다림
      else
      {
        if (prev_collision_obj_.type == OType::STATIC && ego_speed_ == 0)
        {
          static_timer_ += curr_time - prev_static_time_;
          printf("Collision for: %fs\n", static_timer_);
          ROS_WARN("Collision for %fs", static_timer_);
        }
        else
        {
          printf("First time collision\n");
          static_timer_ = 0.0;
        }
        if (static_timer_ <= avoidance_wait_time_thres_)
        {
          printf("Still waiting before avoidance\n");
          is_brake_ = true;
          is_avoiding_ = false;
          is_avoid_ = false;
        }
        else
        {
          printf("Avoidance...\n");
          is_avoid_ = true;
          is_avoiding_ = true;
          is_brake_ = false;
        }
      }
    }
    prev_static_time_ = curr_time;
  }
  // collision with atypical obstacle, always avoid
  else if (obj.type == OType::ATYPICAL)
  {
    is_brake_ = false;
    is_avoid_ = true;
    is_avoiding_ = true;
    static_timer_ = 0.0;
    printf("Collision with ATYPICAL\n");
  }
  // no collision
  else
  {
    printf("No collision\n");
    is_brake_ = false;
    is_avoid_ = false;
    is_avoiding_ = false;
    look_further_ = false;
    static_timer_ = 0.0;
  }
  prev_collision_obj_ = obj;
}

void DynamicPlanning::generateDynamicPath()
{
  // Set safety distance
  safety_distance_ = getSafetyDistance();
  int safety_index = distanceToIndex(sx_, ego_.index, safety_distance_);
  printf("Safety distance: %f\n", safety_distance_);

  // Generate candidate path in s-p coordinate
  s_start_ = sx_.m_x[ego_.index];
  s_end_ = sx_.m_x[safety_index];

  // Generate candidates
  generateCandidates(offset_, safety_index);
  convertPathToCartesian();

  // Calculate costs
  setOffsetCost();
  setObsCost();
  // setContinuityCost();

  // switch among RELAYING I & II & III
  // RELAYING I --> steering: optimal path w/o obstacle, speed: target speed
  // RELAYING II --> steering: optimal path w/o obstacle, speed: u = -3.0
  // RELAYING III --> steering: optimal path w/ obstacle, speed: avoid speed
  switchRelaying();

  // RELAYING I
  if (is_brake_ == false && is_avoid_ == false)
  {
    printf("****************RELAYING I****************\n");
    ROS_WARN("****************RELAYING I****************");
    publishReferencePath(candidates_[opt_path_wo_obs_].points);
    double target_speed = getTargetSpeed(opt_path_wo_obs_);
    visualizePathCandidate(opt_path_wo_obs_);
    setTargetSpeed(target_speed);
  }
  // RELAYING II
  else if (is_brake_ = true && is_avoid_ == false)
  {
    printf("****************RELAYING II****************\n");
    ROS_WARN("****************RELAYING II****************");
    if (!is_avoiding_)
    {
      publishReferencePath(candidates_[opt_path_wo_obs_].points);
      visualizePathCandidate(opt_path_wo_obs_);
    }
    // if ego was avoiding, keep the handle in avoidance
    else
    {
      publishReferencePath(candidates_[opt_path_].points);
      visualizePathCandidate(opt_path_);
    }
    hardBrake();
  }
  // RELAYING III
  else if (is_brake_ == false && is_avoid_ == true)
  {
    is_blocked_ = isBlocked();
    if (!is_blocked_)
    {
      prev_opt_path_ = opt_path_;
      printf("****************RELAYING III****************\n");
      ROS_WARN("****************RELAYING III****************");
      publishReferencePath(candidates_[opt_path_].points);
      setTargetSpeed(20.0);
      visualizePathCandidate(opt_path_);
    }
    else
    {
      printf("****************BLOCKED****************\n");
      ROS_WARN("****************BLOCKED****************");
      // test 필요 (RELAYING III를 가지않고 처음 atypical 까지 땡겼을 때 옆 차선에 차량이 있어서 block이 될 경우 prev_opt_path_ 가 이상해짐)
      publishReferencePath(candidates_[prev_opt_path_].points);
      hardBrake();
    }
  }
  // is_brake_ true && avoid true
  else
  {
    printf("******************UNKNOWN BEHAVIOR************\n");
    return;
  }
}

std::string DynamicPlanning::getLinkID(obj_msgs::Obj obj)
{
  geometry_msgs::Point position;
  printf("get link id: obj idx -> %d, obj id -> %d\n", obj.index, obj.id) ;
  if (obj.type == OType::EGO)
  {
    // position = hood_pos_;
    // position = rear_pos_;
    position = obj.pose.position;
  }
  else
  {
    position = obj.pose.position;
  }
  std::string link_id = global_trajectory_.waypoints[obj.index].CLID;
  assert(!link_id.empty());
  path_msgs::Link clink = links_.at(link_id);
  int clink_idx = searchLinkIndex(position, clink);
  printf("Link id: %s, index: %d\n", link_id.c_str(), clink_idx);
  double cdist = getDistance(position.x,
                            position.y,
                            clink.geometry[clink_idx].x - offsetX_,
                            clink.geometry[clink_idx].y - offsetY_);
  printf("cdist: %f\n", cdist);

  if (cdist >= locate_range_)
  {
    return "";
  }


  if (!clink.RLID[0].empty())
  {
    path_msgs::Link rlink = links_.at(clink.RLID[0]);
    int rlink_idx = searchLinkIndex(position, rlink);
    double rdist = getDistance(position.x,
                              position.y,
                              rlink.geometry[rlink_idx].x - offsetX_,
                              rlink.geometry[rlink_idx].y - offsetY_);
    printf("rdist: %f\n", rdist);
    if (rdist < cdist)
    {
      // printf("AT RIGHT LANE\n");
      link_id = clink.RLID[0];
    }
    else printf("AT CURRENT LANE\n");
  }

  if (!clink.LLID[0].empty())
  {
    path_msgs::Link llink = links_.at(clink.LLID[0]);
    int llink_idx = searchLinkIndex(position, llink);
    double ldist = getDistance(position.x,
                              position.y,
                              llink.geometry[llink_idx].x - offsetX_,
                              llink.geometry[llink_idx].y - offsetY_);
    printf("ldist: %f\n", ldist);
    if (ldist < cdist)
    {
      printf("AT LEFT LANE\n");
      link_id = clink.LLID[0];
    }
    else printf("AT CURRENT LANE\n");
  }

  return link_id;
}

double DynamicPlanning::distBtwObjs(obj_msgs::Obj obj_1, obj_msgs::Obj obj_2)
{
  return (getDistance(obj_1.pose.position.x, obj_1.pose.position.y, obj_2.pose.position.x, obj_2.pose.position.y));
}

// ACC를 하는 조건: 현재 나와 global path의 offset이 1.5 이내이며 obstacle 또한 1.5 이내 일때
// 다른 경우는 무조건 collision avoidance-based로 운행한다.
void DynamicPlanning::executeAdaptiveCruiseControl(obj_msgs::Obj obj)
{
  // Find object's velocity along its path
  // Find object's velocity along its path
  double path_angle;
  double vel_along_path;
  double heading;
  double speed;

  if (obj.type == OType::APPROACHING)
  {
    vel_along_path = 0.0;
  }
  else
  {
    if (obj.index < global_trajectory_.waypoints.size()-1)
    {
      geometry_msgs::Point current_wp = global_trajectory_.waypoints[obj.index].point;
      geometry_msgs::Point next_wp = global_trajectory_.waypoints[obj.index+1].point;
      path_angle= atan2(next_wp.y-current_wp.y, next_wp.x-current_wp.x);
    }
    else
    {
      geometry_msgs::Point current_wp = global_trajectory_.waypoints[obj.index].point;
      geometry_msgs::Point prev_wp = global_trajectory_.waypoints[obj.index-1].point;
      path_angle = atan2(current_wp.y-prev_wp.y, current_wp.x-prev_wp.x);
    }
    heading = atan2(obj.velocity.y, obj.velocity.x);
    speed = sqrt(obj.velocity.y*obj.velocity.y + obj.velocity.x*obj.velocity.x);
    vel_along_path = speed * (1000.0 / 3600.0) * cos(getAngleDiff(heading, path_angle));
    // printf("Heading: %f, tan: %f\n", heading, path_angle);
    printf("Vel along path: %f\n", vel_along_path);
  }

  // Adaptive Cruise Control
  double ref_speed_ms = ref_speed_ * 1000 / 3600.0;
  double d_nominal = getNominalDistance(ref_speed_ms);
  // double c = (27.0 * max_decel_ * max_decel_) / (8.0 * ref_speed_ms * ref_speed_ms * ref_speed_ms);
  double c = (27.0 * max_decel_ * max_decel_) / (8.0 * 9.72222 * 9.72222 * 9.72222);
  double arc_diff = std::get<0>(locateObjectFromEgo(obj));
  printf("ACC -> Arc diff: %f, d_nominal: %f\n", arc_diff, d_nominal);
  printf("ACC -> Headway obj: %f, ego: %f\n", headway_obj_.s, ego_.s);
  double d_tilda = d_nominal - arc_diff;
  double ur;
  double dt = 0.04;
  double diff = distBtwObjs(prev_obj_, obj);
  if (diff > 3*HALF_LANE)
  {
    printf("Prev: %d, Curr: %d\n", prev_obj_.id, obj.id);
    dr_ = arc_diff;
    if (dr_ <= 25.0)
    {
      // Publish acceleration data
      hardBrake();
      printf("Sudden appearance\n");
      // Publish global path as reference path (obj 거리에 따라 더 짧게 publish 할 수 있음 for efficiency)
      std::vector<geometry_msgs::Point> ref_path;
      for(int i = ego_.index; i < global_trajectory_.waypoints.size(); ++i)
      {
        if (i > obj.index)
        {
          break;
        }
        ref_path.push_back(global_trajectory_.waypoints[i].point);
      }
      publishReferencePath(ref_path);
      return;
    }
    count_ = 0;
  }
  else
  {
    dr_ = dr_ + dr_dot_ * dt;
  }
  // printf("Tracked obj: %d\n", obj.id);
  // Find parameters
  d_tilda = d_nominal - dr_;
  // d_tilda 가 처음부터 작으면 (멀리서부터 detect해서 d_nominal과 arc_diff의 차이가 별로 나지 않는다) dr_dot_이 음수가 나와 dr_이 짧아진다.
  // d_tilda 가 처음부터 크면 (갑자기 detect를 하여 d_nominal과 arc_diff의 차이가 많이 나면) dr_dot_이 양수가 나와 dr_이 커진다.
  // i) dt가 너무 작아 dr_dot_이 큼에도 불구하고 dr이 많이 커지지 않아 error가 크다고 판단이 되지 않아 u 값이 크다.
  // ii) dr_dot이 양수가 나올 시 무조건 deceleration을 주는 것이 맞다.
  dr_dot_ = c / 2.0 * d_tilda * d_tilda + vel_along_path - 9.72222;

  // dr_dot_ = c / 2.0 * d_tilda * d_tilda + 0.0 - ref_speed_ms;
  ur = c * abs(d_tilda) * dr_dot_;
  // min_arc 아닐 수도...
  double error = dr_ - arc_diff;
  double u = ur - 0.2 * error ;

  // printf("It: %d -> dr: %f, d_tilda: %f, dr_dot: %f, ur: %f\n", count_, dr_, d_tilda, dr_dot_, ur);
  // Distance to obstacle is smaller than collision distance
  // prev_obj_id_ = obj.id;
  prev_obj_ = obj;
  count_++;

  // Object's distance is further than d nominal
  if (d_tilda < 0)
  {
    count_ = 0;
    // prev_obj_id_ = 0;
    obj_msgs::Obj reset;
    prev_obj_ = reset;
    printf("Further than d nominal\n");
    return;
  }

  printf("It: %d -> dr: %f c: %f d_tilda: %f dr_dot: %f ur: %f, error: %f, u: %f\n", count_, dr_, c, d_tilda, dr_dot_, ur, error, u);

  double max_accel;
  if (ego_speed_ < 3)
  {
    max_accel = 0.5;
  }
  else
  {
    max_accel = 0.2;
  }

  if (u < max_decel_)
  {
    printf("AVOID!!!\n");
    // u = max_decel_;
  }
  else if (u > max_accel)
  {
    u = max_accel;
  }

  // Publish global path as reference path (obj 거리에 따라 더 짧게 publish 할 수 있음 for efficiency)
  std::vector<geometry_msgs::Point> ref_path;
  for(int i = ego_.index; i < global_trajectory_.waypoints.size(); ++i)
  {
    if (i > obj.index)
    {
      break;
    }
    ref_path.push_back(global_trajectory_.waypoints[i].point);
  }
  publishReferencePath(ref_path);

  if (dr_dot_ > 10)
  {
    u = max_decel_;
  }

  // Publish acceleration data
  std_msgs::Float64 msg;
  msg.data = u;
  ego_acc_pub_.publish(msg);
}

void DynamicPlanning::getBezierInfo()
{
  beziers_.clear();
  double length = 0.0;
  for(int i = 0; i < global_trajectory_.waypoints.size() - 1; i++)
  {
    path_msgs::Bezier bezier;
    path_msgs::Waypoint curr_wp = global_trajectory_.waypoints[i];
    path_msgs::Waypoint next_wp = global_trajectory_.waypoints[i+1];

    if (curr_wp.type == 1)
    {
      bezier.start_idx = i;
      if (next_wp.type == 1)
      {
        length += getDistance(curr_wp.point.x, curr_wp.point.y, next_wp.point.x, next_wp.point.y);
      }
      else
      {
        bezier.length = length;
        beziers_.push_back(bezier);
        length = 0.0;
      }
    }
  }
  for (int i = 0; i < beziers_.size(); i++)
  {
    printf("bezier %d, start_idx: %d, length: %f\n", i, beziers_[i].start_idx, beziers_[i].length);
  }
}

double DynamicPlanning::getBezierSpeed()
{
  int start_point = 0;
  bool is_bezier = false;
  double length = 0.0;
  for(int i = ego_.index; i < global_trajectory_.waypoints.size()-1; i++)
  {
    path_msgs::Waypoint curr_wp = global_trajectory_.waypoints[i];
    path_msgs::Waypoint next_wp = global_trajectory_.waypoints[i+1];
    // Waypoint is within bezier curve
    if (curr_wp.type == 1)
    {
      if (!is_bezier)
      {
        start_point = i;
        is_bezier = true;
      }
      if (next_wp.type == 1)
      {
        length += getDistance(curr_wp.point.x, curr_wp.point.y, next_wp.point.x, next_wp.point.y);
      }
      else
      {
        break;
      }
    }
  }
  printf("Bezier length: %f\n", length);
  // Set speed if bezier is found
  if (is_bezier)
  {
    double distance = sx_.m_x[start_point] - sx_.m_x[ego_.index];
    double speed = global_trajectory_.waypoints[start_point].speed / 2.0;
    // double speed_diff = ego_speed_ - speed;
    double speed_diff = (ego_speed_ - speed) * 1000.0 / 3600.0;
    double steady_state_time = 6.0;
    double threshold = 20.0;
    if (speed_diff > 0)
    {
      threshold = steady_state_time * speed_diff;
    }
    printf("Bezier threshold: %f, distance: %f\n", threshold, distance);
    if (distance < threshold)
    {
      printf("Bezier speed: %f\n", speed);
      return speed;
    }
    else
    {
      return ref_speed_;
    }
  }
  else
  {
    return ref_speed_;
  }
}


double DynamicPlanning::getSpeedLimit()
{
  path_msgs::Link current_link = links_.at(ego_.link_id);
  // Currently not at the turn, take next link's speed
  // printf("Current link type: %d\n", current_link.road_type);
  if (current_link.road_type != RType::TURN && current_link.road_type != RType::ATYPICAL_TURN)
  {
    for(int i=ego_.index; i < global_trajectory_.waypoints.size(); i++)
    {
      if (global_trajectory_.waypoints[i].CLID == searchNextLink(ego_.index))
      {
        path_msgs::Link next_link = links_.at(global_trajectory_.waypoints[i].CLID);
        // printf("%s\n", global_trajectory_.waypoints[i].CLID.c_str());
        double distance = sx_.m_x[i] - sx_.m_x[ego_.index];
        double speed = global_trajectory_.waypoints[i].speed;
        // double speed_diff = ego_speed_ - speed;
        // double steady_state_time = 4.0;
        double speed_diff = (ego_speed_ - speed) * 1000.0 / 3600.0;
        printf("Ego speed: %f, target speed: %f\n", ego_speed_, speed);
        double steady_state_time = 8.0;
        double threshold = 20.0;
        if (next_link.road_type == RType::ATYPICAL_TURN)
        {
            threshold = 20.0;
        }
        if (speed_diff > 0)
        {
          threshold = steady_state_time * speed_diff;
        }
        printf("Threshold %f, distance: %f\n", threshold, distance);
        if (distance < threshold)
        {
          printf("Speed: %f\n", speed);
          return speed;
        }
        else
        {
          break;
        }
      }
    }
  }
  // At the turn, take current link's speed
  else
  {
    printf("Following current link's speed\n");
    return current_link.speed;
  }
  printf("Following current link's speed\n");
  return current_link.speed;
}

double DynamicPlanning::getCurvatureSpeed(int index)
{
  return sqrt(lat_acc_limit_ / (candidates_[index].max_k*1000.0));
}

double DynamicPlanning::getAvoidSpeed(int index)
{
  if (obj_ahead_ && !(is_blocked_ || is_deadend_))
  {
    bool is_veh = isVehicle(headway_obj_);
    if (is_veh)
    {
      return 20.0;
    }
    else
    {
      return ref_speed_;
    }
  }
  else
  {
    return ref_speed_;
  }
  // return (1 - safety_gain_ * pow(candidates_[index].oc, 2)) * ref_speed_ * (5.0 / 7.0);
}

double DynamicPlanning::getEndOfPathSpeed()
{
  bool reached = isReached();
  return reached ? 0.0 : ref_speed_;
}

double DynamicPlanning::getTargetSpeed(int index)
{
  std::vector<double> speeds;
  // Speed based on curvature
  double curv_v = getCurvatureSpeed(index);
  // Speed based on the static cost of a path (avoidance)
  // double avoid_v = getAvoidSpeed(index);
  // sign limit
  double limit_v = getSpeedLimit();
  // Speed for the end of planned trajectory
  // double stop_v = getEndOfPathSpeed(); // ACC 변환?
  // Speed for bezier curve
  double bezier_v = getBezierSpeed();
  // cut-in speed if we goes into the global path from outside
  double cutin_v = getCutInSpeed();
  speeds.push_back(curv_v);
  // speeds.push_back(avoid_v);
  // speeds.push_back(stop_v);
  speeds.push_back(limit_v);
  speeds.push_back(bezier_v);
  speeds.push_back(cutin_v);
  double min_speed = *min_element(speeds.begin(), speeds.end());
  printf("curv_v: %f\nbezier_v: %f\nlimit_v: %f\n min: %f\n", curv_v, bezier_v,limit_v,min_speed);
  // printf("curv_v: %f\navoid_v: %f\nbezier_v: %f\nlimit_v: %f\n min: %f\n", curv_v,avoid_v,bezier_v,limit_v,min_speed);
  return min_speed;
}

double DynamicPlanning::getCutInSpeed()
{
  bool is_on_path = isOnPath();
  if (is_on_path)
  {
    return ref_speed_;
  }
  else
  {
    return 15.0;
  }
}

Candidate DynamicPlanning::generatePathCandidate(double offset, int safety_index)
{
  Candidate cand;
  tk::spline sp;
  std::vector<double> p(2); std::vector<double> s(2);
  p[0] = ego_.lat_offset;
  p[1] = ego_.lat_offset + offset;
  s[0] = s_start_;
  s[1] = s_end_;
  // ROS_INFO("p1: %f, p2: %f, s_start: %f, s_end: %f", p[0], p[1], s[0], s[1]);
  sp.set_boundary(tk::spline::first_deriv, tan(relative_yaw_), tk::spline::first_deriv, 0, false);
  sp.set_points(s,p);
  cand.sp = sp;
  // ROS_INFO("%f %f %f", sp.m_a[0], sp.m_b[0], sp.m_c[0]);
  return cand;
}


void DynamicPlanning::convertPathToCartesian()
{
  // // Get current heading and position
  // double cx = ego_.pose.position.x + gps_to_hood_ * cos(yaw_);
  // double cy = ego_.pose.position.y + gps_to_hood_ * sin(yaw_);

  double cx = ego_.pose.position.x;
  double cy = ego_.pose.position.y;

  // double cx = ego_.pose.position.x - gps_to_rear_ * cos(yaw_);
  // double cy = ego_.pose.position.y - gps_to_rear_ * sin(yaw_);

  // Convert from s-p to cartesian
  double ds = 0.5;
  for(int i=0; i < candidates_.size(); i++)
  {
    tk::spline cand_sp = candidates_[i].sp;
    bool is_lah = false;
    double point_x = cx;
    double point_y = cy;
    double yaw = yaw_;
    double curv_cost = 0.0;
    double max_k = 0.0;
    double sum_k = 0.0;

    for(double s0 = s_start_; s0 <= s_end_; s0 = s0 + ds)
    {
      double x0 = sx_(s0); double y0 = sy_(s0);
      double x1 = sx_.deriv(1,s0); double y1 = sy_.deriv(1,s0);
      double x2 = sx_.deriv(2,s0); double y2 = sy_.deriv(2,s0);
      // double k0 = sqrt(pow(x2,2) + pow(y2,2));
      double k0 = (x1*y2-x2*y1) / pow(x1*x1 + y1*y1, 1.5);
      sum_k += k0;
      double p0 = cand_sp(s0); double p1 = cand_sp.deriv(1,s0); double p2 = cand_sp.deriv(2,s0);
      // k0 = abs(xy.deriv(2, x0) / pow(1+xy.deriv(1,x0)*xy.deriv(1,x0), 1.5));
      // printf("xY1: %f, xY2: %f, K0: %f\n", xy.deriv(1, xprev), xy.deriv(2, xprev),k0);
      // k0 = abs((x1*y2-x2*y1) / pow(x1*x1 + y1*y1, 1.5));
      // printf("x1: %f, x2: %f, y1: %f, y2: %f, k0: %f\n", x1, x2, y1, y2, k0);
      double A = sqrt(p1*p1 + (1-p0*k0)*(1-p0*k0));
      double B = ((1-p0*k0) > 0 ? 1 : -1);
      if (1-p0*k0 == 0) {B=0;}
      double k = B/A * (k0 + ((1-p0*k0)*p2 + k0*p1*p1)/(A*A));
      // ROS_INFO("A: %f", A);
      // ROS_INFO("B: %f", B);
      // printf("p0: %f, p1 %f, p2 %f\n", p0, p1, p2);

      // Find lookahead point on the path
      // printf("Cand: %d, diff: %f, lookahead_distance: %f\n", i, s0-s_start_, lookahead_distance_);

      if (s0-s_start_ >= lookahead_distance_ && !is_lah)
      {
        geometry_msgs::Point lah_point;
        lah_point.x = point_x;
        lah_point.y = point_y;
        candidates_[i].lah_point = lah_point;
        is_lah = true;
      }
      else if (s0-s_start_ >= 8.0 && !is_lah)
      {
        geometry_msgs::Point lah_point;
        lah_point.x = point_x;
        lah_point.y = point_y;
        candidates_[i].lah_point = lah_point;
        is_lah = true;
      }

      // Insert point
      geometry_msgs::Point point;
      point.x = point_x;
      point.y = point_y;
      candidates_[i].points.push_back(point);

      // ROS_INFO("(%f, %f, %f)", temp_x, temp_y, temp_yaw);
      // Propagate to next step
      point_x = point_x + A*B*cos(yaw)*ds;
      point_y = point_y + A*B*sin(yaw)*ds;
      yaw = yaw + A*B*k*ds;

      // printf("k: %f, delta x: %f, delta y: %f, delta theta: %f, k0: %f, A: %f, B: %f\n", k, A*B*cos(temp_yaw)*ds, A*B*sin(temp_yaw)*ds , A*k*ds, k0, A, B);
      // Find max k
      if (abs(k) > max_k)
      {
        max_k = abs(k);
      }
      // Smoothness cost
      curv_cost += k*k*ds;
    }
    // printf("Candidate %d --> k: %f\n", i, max_k);
    // printf("Total k: %f\n", sum_k);
    candidates_[i].max_k = max_k;
    candidates_[i].smc = curv_cost;
    // ROS_INFO("%d: %f", i, curv_squared_sum);
  }
}

// int DynamicPlanning::selectOptimalPath()
// {
//   double min_cost = 10000.0;
//   int min_index = 0;
//   // printf("smoothness, static, offset\n");
//   for(int i=0; i<candidates_.size();i++)
//   {
//     double cost = wc_*(a_ * candidates_[i].smc + b_*candidates_[i].coc) + ws_*candidates_[i].oc + wl_*candidates_[i].gc;
//     if (cost < min_cost)
//     {
//       min_index = i;
//       min_cost = cost;
//     }
//     printf("%d cost: %3f, %3f, %3f --> %3f, %1f\n",
//     i,
//     wc_*(a_ * candidates_[i].smc + b_*candidates_[i].coc),
//     ws_*candidates_[i].oc,
//     wl_*candidates_[i].gc,
//     cost, R_[i]);
//   }
//   // printf("Optimal path: %d\n", min_index);
//   return min_index;
// }

void DynamicPlanning::visualizePathCandidate(int optimal_index)
{

  // Visualize path candidates
  for(int i=0; i < candidates_.size(); i++)
  {
    visualization_msgs::Marker rviz_path;
    rviz_path.header.frame_id = map_frame_;
    rviz_path.header.stamp = ros::Time::now();
    rviz_path.action = visualization_msgs::Marker::ADD;
    rviz_path.pose.orientation.w = 1.0;
    rviz_path.id = i;
    rviz_path.type = visualization_msgs::Marker::LINE_STRIP;
    rviz_path.scale.x = 0.05;
    rviz_path.color.r = 0.0;
    rviz_path.color.g = 1.0;
    rviz_path.color.b = 0.0;
    rviz_path.color.a = 1.0;
    //rviz_path.lifetime = ros::Duration(0.2);

    // No way to go
    if (R_all_[i] >= 0.5)
    {
      rviz_path.color.r = 1.0;
      rviz_path.color.g = 0.0;
      rviz_path.color.b = 0.0;
    }

    for(int j=0; j < candidates_[i].points.size(); j++)
    {
      // geometry_msgs::Point point;
      // point.x = std::get<0> (candidates_[i].points[j]);
      // point.y = std::get<1> (candidates_[i].points[j]);
      rviz_path.points.push_back(candidates_[i].points[j]);
    }

    if (i == optimal_index)
    {
      rviz_path.color.r = 0.0;
      rviz_path.color.g = 0.0;
      rviz_path.color.b = 1.0;
    }
    publishers_.at(i).publish(rviz_path);
  }
}

void DynamicPlanning::publishReferencePath(std::vector<geometry_msgs::Point> path)
{
  visualization_msgs::Marker rviz_path;
  rviz_path.header.frame_id = map_frame_;
  rviz_path.header.stamp = ros::Time::now();
  rviz_path.action = visualization_msgs::Marker::ADD;
  rviz_path.pose.orientation.w = 1.0;
  rviz_path.id = 999;
  rviz_path.type = visualization_msgs::Marker::LINE_STRIP;
  rviz_path.scale.x = 0.2;
  rviz_path.color.r = 0.0;
  rviz_path.color.g = 0.0;
  rviz_path.color.b = 1.0;
  rviz_path.color.a = 1.0;

  for(int i = 0; i < path.size(); i++)
  {
    rviz_path.points.push_back(path[i]);
  }
  ref_path_pub_.publish(rviz_path);
}

bool DynamicPlanning::isVehicleAround()
{
  for(int i = 0; i < veh_list_.size(); i++)
  {
    if (veh_list_[i].type != OType::ATYPICAL)
    {
      return true;
    }
  }
  return false;
}

void DynamicPlanning::searchObjAhead()
{
  std::vector<obj_msgs::Obj> obj_list;
  obj_list.insert(obj_list.end(), veh_list_.begin(), veh_list_.end());
  obj_list.insert(obj_list.end(), traffic_list_.begin(), traffic_list_.end());
  obj_list.insert(obj_list.end(), stop_nodes_.begin(), stop_nodes_.end());
  obj_list.push_back(goal_);
  double min_arc = 9999;
  for(int i = 0; i < obj_list.size(); i++)
  {
    obj_msgs::Obj obj = obj_list[i];
    std::tuple<double, double> obj_info = locateObjectFromEgo(obj);
    double arc_diff = std::get<0>(obj_info);

    // Check whether objects share the same lane
    // if(abs(ego_.lat_offset) <= 1.5 && abs(obj.lat_offset) <= 1.5)
    // if abs(obj.lat_offset <= 1.5)
    // {
    // Temp
    // if (abs(obj.lat_offset <= 1.5))
    // {

    // if (abs(obj.lat_offset <= 1.5))를 넣으면 global path에 있는 obstacle만 보게 됨
    // 실제로는 앞에 있는 모든 obstacle을 찾아야함
    if (abs(obj.lat_offset) <= HALF_LANE)
    {
      if (arc_diff < min_arc && arc_diff > gps_to_hood_)
      {
        min_arc = arc_diff;
        headway_obj_ = obj;
      }
    }
  }
  if (headway_obj_.id == 0)
  {
    obj_ahead_ = false;
  }
  else
  {
    obj_ahead_ = true;
  }
  printf("Headway obj: %d, index: %d, velocity x: %f, velocity y: %f\n", headway_obj_.id, headway_obj_.index, headway_obj_.velocity.x, headway_obj_.velocity.y);
}

void DynamicPlanning::printObjStatus()
{
  printf("==============================%d Vehicles===========================\n", veh_list_.size());
  for(int i = 0; i < veh_list_.size(); i++)
  {
    obj_msgs::Obj obj = veh_list_[i];
    printf("Veh %d : %s, index %d, arc %f, lat offset %f m, vel_x: %f, vel_y: %f\n", obj.id, obj.link_id.c_str(), obj.index, obj.s, obj.lat_offset, obj.velocity.x, obj.velocity.y);
  }
  printf("================================================================\n");
  printf("==============================Traffic Light===========================\n");
  for(int i = 0; i < traffic_list_.size(); i++)
  {
    obj_msgs::Obj traffic_light = traffic_list_[i];
    printf("Traffic light: id: %d, index %d, arc %f, lat offset %f m\n", traffic_light.id, traffic_light.index, traffic_light.s, traffic_light.lat_offset);
  }
  printf("================================================================\n");
  printf("==============================Stop Node===========================\n");
  for(int i = 0; i < stop_nodes_.size(); i++)
  {
    obj_msgs::Obj node = stop_nodes_[i];
    printf("Stop node id: %d: index %d, arc %f, lat offset %f m\n", node.id, node.index, node.s, node.lat_offset);
  }
  printf("================================================================\n");
}

double DynamicPlanning::getNominalDistance(double ref_speed_ms)
{
  double dc;
  if (ego_speed_ <= 15)
  {
    dc = 20;
  }
  else if (ego_speed_ <= 20)
  {
    dc = 20 + (ego_speed_ - 15) * 0.4;
  }
  else if(ego_speed_ <= 25)
  {
    dc = 22 + (ego_speed_-20)* 0.6;
  }
  else if(ego_speed_ <= 35)
  {
    dc = 25 + ego_speed_- 25;
  }
  else
  {
    dc = 36;
  }
  return sqrt(16.0/27.0) * 9.72222 * 9.72222 / abs(max_decel_) + dc;
}

void DynamicPlanning::updateObjAhead()
{
  if (is_deadend_)
  {
    if (obj_deadend_.s < headway_obj_.s)
    {
      printf("Deadend object ahead\n");
      headway_obj_ = obj_deadend_;
      obj_ahead_ = true;
    }
  }
}
//
// void DynamicPlanning::updateObjAhead()
// {
//   bool is_veh = isVehicleAround();
//   if (is_veh)
//   {
//     // Add the object that induces the dead-end
//     if (is_deadend_)
//     {
//       if (obj_deadend_.s < headway_obj_.s)
//       {
//         printf("Deadend object ahead\n");
//         headway_obj_ = obj_deadend_;
//         obj_ahead_ = true;
//       }
//     }
//     // Look for object that collides with global-path-following path
//     if (state_ == DynamicPlanning::STATE::RELAYING)
//     {
//       // Search for object ahead on the locally planned path
//       // Collision with approaching vehicle (not located)
//       Candidate cand = getPathFollowingCandidate();
//       obj_msgs::Obj obj = cand.collision_obj;
//       if (obj.type == OType::APPROACHING)
//       {
//         // dtc: distance-to-point-of-collision
//         double obj_speed = sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y*obj.velocity.y);
//         double obj_dtpoc = obj_speed * cand.time_to_collision;
//         double ego_speed_ms = ego_speed_ * 1000.0 / 3600.0;
//         double ego_dtpoc = ego_speed_ms * cand.time_to_collision;
//         printf("TTC: %f\n", cand.time_to_collision);
//         ROS_WARN("obj's speed: %f, distance-to-point-of-collision: %f", obj_speed, obj_dtpoc);
//         printf("obj's speed: %f, distance-to-point-of-collision: %f\n", obj_speed, obj_dtpoc);
//         ROS_WARN("Ego's speed: %f, distance-to-point-of-collision: %f", ego_speed_ms, ego_dtpoc);
//         printf("Ego's speed: %f, distance-to-point-of-collision: %f\n", ego_speed_ms, ego_dtpoc);
//
//         // 만약 현재 차량 속도로 ttc 시간 만큼 주행 했을 경우의 거리에 safety distance를 더한 것이
//         // object가 ttc 시간만큼 주행했을 때 거리보다 짧을 경우에만 ACC를 진행하도록 전방 물체를 만든다.
//         // if (ego_dtpoc - obj_dtpoc <= safety_distance_)
//         if (ego_dtpoc - obj_dtpoc <= cut_in_thres_)
//         {
//           obj.pose.position.x = cand.collision_point.x;
//           obj.pose.position.y = cand.collision_point.y;
//           locateObj(obj);
//         }
//         else
//         {
//           printf("Not within the cut-in threshold\n");
//           obj.index = -1;
//         }
//       }
//       if (obj.index >= 0 && obj.link_id !="")
//       {
//         printf("Path following obj %d: type: %d, arc: %f, index: %d\n", obj.id, obj.type, obj.s, obj.index);
//         // 멀리서 본 headway obj보다 가까지 있다면
//         if (obj.s < headway_obj_.s)
//         {
//           headway_obj_ = obj;
//           obj_ahead_ = true;
//         }
//       }
//     }
//   }
// }

// If any path +- 5 index from path following path index has avoidable object,
// return collision object's type
Candidate DynamicPlanning::getPathFollowingCandidate()
{
  int min_idx = 0;
  double min_cost = candidates_[min_idx].gc;
  for(int i = 0; i < candidates_.size(); i++)
  {
    double cost = candidates_[i].gc;
    if (cost < min_cost)
    {
      min_cost = cost;
      min_idx = i;
    }
  }
  return candidates_[min_idx];
  // for(int i = 0; i <= 5; i++)
  // {
  //   Cand upper_candidate;
  //   Cand lower_candidate;
  //   if (path_following_index + i < candidates_.size())
  //   {
  //     upper_candidate = candidates_[path_following_index + i];
  //   }
  //   if (path_following_index - i >= 0)
  //   {
  //     lower_candidate = candidates_[path_following_index - i];
  //   }
  //
  //   if (path_following_candidate.collision_obj.type == OTYPE::APPROACHING ||
  //     upper_candidate.collision_obj.type == OTYPE::APPROACHING ||
  //     lower_candidate.collision_obj.type == OTYPE::APPROACHING)
  //   {
  //     return OType::APPROACHING;
  //   }
  //   else if (path_following_candidate.collision_obj.type == OTYPE::STATIC ||
  //           upper_candidate.collision_obj.type == OTYPE::STATIC ||
  //           lower_candidate.collision_obj.type == OTYPE::STATIC)
  //   {
  //     return OType::STATIC;
  //   }
  //   else
  //   {
  //     return OType::UNKNOWN;
  //   }
  // }
}

bool DynamicPlanning::collisionWithApproachingObject(double checking_range)
{
  // Find reference index based on the checking range
  int ref_index = 0;
  bool found = false;
  bool is_obs = false;

  for(int i = ego_.index; i < global_trajectory_.waypoints.size(); i++)
  {
    double distance = sx_.m_x[i] - sx_.m_x[ego_.index];
    if (distance > checking_range)
    {
      found = true;
      ref_index = i;
    }
  }
  if (!found)
  {
    ref_index = global_trajectory_.waypoints.size() - 1;
  }

  // Check appraoching objects
  for(int i = 0; i < veh_list_.size(); i++)
  {
    obj_msgs::Obj obj = veh_list_[i];
    if (obj.type == OType::APPROACHING || obj.type == OType::FRONTOBJ)
    {
      for (int j = ego_.index; j <= ref_index; j++)
      {
        double x = global_trajectory_.waypoints[j].point.x;
        double y = global_trajectory_.waypoints[j].point.y;
        for (double t=0.0; t <= 10.0; t+= 0.1)
        {
          obj_msgs::Obj tracker = obj;
          if (is_rect_)
          {
            tracker.ul.x = obj.ul.x + t * obj.velocity.x;
            tracker.ul.y = obj.ul.y + t * obj.velocity.y;
            tracker.ur.x = obj.ur.x + t * obj.velocity.x;
            tracker.ur.y = obj.ur.y + t * obj.velocity.y;
            tracker.bl.x = obj.bl.x + t * obj.velocity.x;
            tracker.bl.y = obj.bl.y + t * obj.velocity.y;
          }
          else if (is_ellipse_)
          {
            tracker.alpha = obj.alpha;
            tracker.a = obj.a;
            tracker.b = obj.b;
            tracker.pose.position.x = obj.pose.position.x + t * obj.velocity.x;
            tracker.pose.position.y = obj.pose.position.y + t * obj.velocity.y;
          }
          if (isCollision(x, y, tracker))
          {
            printf("<<<GLOBAL>>>\n");
            printf("Approaching: collision..., index: %d, type: %d, id: %d\n", obj.index, obj.type, obj.id);
            printf("Collision point -> x: %f, y: %f, ttc: %f\n", x, y, t);
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool DynamicPlanning::isOnPath()
{
  if (abs(ego_.lat_offset) <= HALF_LANE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void DynamicPlanning::switchState()
{
  if (obj_ahead_)
  {
    double ref_speed_ms = ref_speed_ * 1000.0 / 3600.0;
    double arc_diff = std::get<0>(locateObjectFromEgo(headway_obj_));
    double d_nominal = getNominalDistance(ref_speed_ms);
    double d_tilda = d_nominal - arc_diff;
    printf("Headway obj: %d, arc: %f, index: %d\n", headway_obj_.id, headway_obj_.s, headway_obj_.index);
    printf("Arc diff: %f, d nominal: %f\n", arc_diff, d_nominal);

    bool is_on_path = isOnPath();
    bool is_acc_done = isAdaptiveCruiseControlFinish();

    // Check wheter ACC to a vehicle is finished (enough waiting for avoidance time)
    if (is_acc_done)
    {
      printf("Finish ACC... start avoiding\n");
      ROS_WARN("Finish ACC... start avoiding\n");
      acc_timer_ = 0.0;
      state_ = DynamicPlanning::STATE::RELAYING;
    }
    // Still in CRUISING or o.w
    else
    {
      // Currently avoiding
      if (is_avoid_)
      {
        printf("Avoiding in dynamic\n");
        ROS_WARN("Avoiding in dynamic");
        state_ = DynamicPlanning::STATE::RELAYING;
      }
      else
      {
        // Inside the global path
        if (is_on_path)
        {
          // 뒤에서 오는 approaching 거르기 ?
          // check collision with approaching object
          bool is_collision_front = collisionWithApproachingObject(arc_diff);
          // bool is_collision_bezier;
          if (is_collision_front)
          {
            printf("Collision in the global path --> front: %d, bezier: %d\n", is_collision_front);
            ROS_WARN("Collision in the global path --> front: %d, bezier: %d", is_collision_front);
            state_ = DynamicPlanning::STATE::HARD_BRAKE;
          }
          else
          {
            // Within ACC region --> CRUISING
            if (d_tilda >= 0)
            {
              printf("No collision in the global path\n");
              ROS_WARN("No collision in the global path");
              state_ = DynamicPlanning::STATE::CRUISING;
            }
            // Outside ACC region --> RELAYING
            else
            {
              printf("Outside ACC region\n");
              ROS_WARN("Outside ACC region");
              // avoid after 10 seconds depending on the situation
              state_ = DynamicPlanning::STATE::RELAYING;
            }
          }
        }
        // Outside the global path
        else
        {
          printf("Outside the global path\n");
          ROS_WARN("Outside the global path");
          state_ = DynamicPlanning::STATE::RELAYING;
        }
      }
    }
  }
  else
  {
    state_ = DynamicPlanning::STATE::RELAYING;
  }
}

//
// void DynamicPlanning::switchState()
// {
//   if (obj_ahead_)
//   {
//     double ref_speed_ms = ref_speed_ * 1000.0 / 3600.0;
//     double arc_diff = std::get<0>(locateObjectFromEgo(headway_obj_));
//     double d_nominal = getNominalDistance(ref_speed_ms);
//     double d_tilda = d_nominal - arc_diff;
//     printf("Headway obj: %d, arc: %f, index: %d\n", headway_obj_.id, headway_obj_.s, headway_obj_.index);
//     printf("Arc diff: %f, d nominal: %f\n", arc_diff, d_nominal);
//
//     bool is_on_path = isOnPath();
//     bool is_acc_done = isAdaptiveCruiseControlFinish();
//
//     // Check wheter ACC to a vehicle is finished (enough waiting for avoidance time)
//     if (is_acc_done)
//     {
//       printf("Finish ACC... start avoiding\n");
//       ROS_WARN("Finish ACC... start avoiding\n");
//       acc_timer_ = 0.0;
//       state_ = DynamicPlanning::STATE::RELAYING;
//     }
//     // Still in CRUISING or o.w
//     else
//     {
//       // within ACC region
//       if (d_tilda >= 0)
//       {
//         printf("Within ACC region\n");
//         ROS_WARN("Within ACC region");
//         // check avoiding within ACC
//         if (is_avoid_)
//         {
//           // DO avoid after 10 seconds
//           printf("Avoid within ACC region\n");
//           ROS_WARN("Avoid within ACC region");
//           // look_further_ = true;
//           state_ = DynamicPlanning::STATE::RELAYING;
//         }
//         else
//         {
//           // Inside the global path
//           if (is_on_path)
//           {
//             // 뒤에서 오는 approaching 거르기 ?
//             // check collision with approaching object
//             bool is_collision_front = collisionWithApproachingObject(arc_diff);
//             // bool is_collision_bezier;
//             if (is_collision_front)
//             {
//               printf("Collision in the global path --> front: %d, bezier: %d\n", is_collision_front, is_collision_bezier);
//               ROS_WARN("Collision in the global path --> front: %d, bezier: %d", is_collision_front, is_collision_bezier);
//               state_ = DynamicPlanning::STATE::HARD_BRAKE;
//             }
//             else
//             {
//               printf("No collision in the global path\n");
//               ROS_WARN("No collision in the global path");
//               state_ = DynamicPlanning::STATE::CRUISING;
//             }
//           }
//           // outside the global path
//           // 필요한 이유: 비정형은 ACC가 아니므로 headway object로 안 잡히는 상황에서 바로 앞에 골이 있으면 그냥 가버릴 수 있음
//           else
//           {
//             printf("Outside the global path\n");
//             ROS_WARN("Outside the global path");
//             state_ = DynamicPlanning::STATE::RELAYING;
//           }
//         }
//       }
//       // Outside ACC region
//       else
//       {
//         printf("Outside ACC region\n");
//         ROS_WARN("Outside ACC region");
//         // avoid after 10 seconds depending on the situation
//         state_ = DynamicPlanning::STATE::RELAYING;
//       }
//     }
//   }
//   else
//   {
//     state_ = DynamicPlanning::STATE::RELAYING;
//   }
// }

bool DynamicPlanning::isSafeLane(path_msgs::Lane lane)
{
  // Lane is not a center line and not a curb
  if (state_ == DynamicPlanning::STATE::RELAYING)
  {
    return (lane.lane_code != LCode::BARRIER &&
      lane.lane_code != LCode::CENTER &&
      lane.lane_code != LCode::UTURN &&
      lane.lane_code != LCode::NO_LANE_CHANGE &&
      lane.lane_code != LCode::NO_PARKING &&
      lane.lane_code != LCode::NO_STOPPING);
  }
  else if (state_ == DynamicPlanning::STATE::CRUISING)
  {
    return (lane.lane_code != LCode::BARRIER
      && lane.lane_code != LCode::CENTER
      && lane.lane_code != LCode::UTURN
      && lane.lane_code != LCode::NO_LANE_CHANGE
      && lane.lane_code != LCode::LANE
      && lane.lane_code != LCode::NO_PARKING
      && lane.lane_code != LCode::NO_STOPPING);
  }
}


bool DynamicPlanning::isAtIntersection(int index)
{
  if (global_trajectory_.waypoints[index].CLID.find("LL") != std::string::npos)
  {
    ROS_INFO_STREAM("At the left-turn link.");
    return true;
  }
  else if (global_trajectory_.waypoints[index].CLID.find("RR") != std::string::npos)
  {
    ROS_INFO_STREAM("At the right-turn link.");
    return true;
  }
}

bool DynamicPlanning::isDeadEnd()
{
  obj_msgs::Obj reset;
  obj_deadend_ = reset;
  for(int i=0; i < raw_veh_list_.size(); i++)
  {
    obj_msgs::Obj obj = raw_veh_list_[i];
    if (obj.s >= ego_.s)
    {
      obj_msgs::Obj bl;
      obj_msgs::Obj br;
      obj_msgs::Obj ul;
      obj_msgs::Obj ur;
      bl.pose.position.x = obj.bl.x;
      bl.pose.position.y = obj.bl.y;
      bl.index = obj.index;
      br.pose.position.x = obj.br.x;
      br.pose.position.y = obj.br.y;
      br.index = obj.index;
      ul.pose.position.x = obj.ul.x;
      ul.pose.position.y = obj.ul.y;
      ul.index = obj.index;
      ur.pose.position.x = obj.ur.x;
      ur.pose.position.y = obj.ur.y;
      ur.index = obj.index;

      double bl_lat = std::get<1>(getSP(bl));
      double br_lat = std::get<1>(getSP(br));
      double ul_lat = std::get<1>(getSP(ul));
      double ur_lat = std::get<1>(getSP(ur));

      LControl lane_status = laneStatus(links_.at(obj.link_id));

      // Object within global path
      if (abs(obj.lat_offset) <= HALF_LANE) // 1.5
      {
        // Both adjacnet links are available
        if (lane_status == LControl::BOTH)
        {
          // both left side and right side of the box crosses the adjacent links
          if ((bl_lat > HALF_LANE || ul_lat > HALF_LANE) && (ur_lat < -HALF_LANE || br_lat < -HALF_LANE))
          {
            ROS_WARN("Deadend: all three links blocked");
            if (obj.s <= obj_deadend_.s)
            {
              obj_deadend_ = obj;
            }
          }
        }
        // Only right link is available
        else if (lane_status == LControl::RIGHT)
        {
          // right side of the box crosses the right link
          if(br_lat < -HALF_LANE || ur_lat < -HALF_LANE)
          {
            ROS_WARN("Deadend: right and current links are blocked");
            if (obj.s <= obj_deadend_.s)
            {
              obj_deadend_ = obj;
            }
          }
        }
        // Only left link is available
        else if (lane_status == LControl::LEFT)
        {
          // left side of the box crosses the left link
          if(bl_lat > HALF_LANE || ul_lat > HALF_LANE)
          {
            ROS_WARN("Deadend: left and current links are blocked");
            if (obj.s <= obj_deadend_.s)
            {
              obj_deadend_ = obj;
            }
          }
        }
        // Only current link is available
        else if (lane_status == LControl::KEEP)
        {
          ROS_WARN("Deadend: current link is blocked");
          if (obj.s <= obj_deadend_.s)
          {
            obj_deadend_ = obj;
          }
        }
      }
      // Outside the global path
      else
      {
        // Either side of the box crosses the current global link
        if (abs(bl_lat) < HALF_LANE || abs(ul_lat) < HALF_LANE || abs(br_lat) < HALF_LANE || abs(ur_lat) < HALF_LANE)
        {
          ROS_WARN("Deadend: blocking the current link");
          if (obj.s <= obj_deadend_.s)
          {
            obj_deadend_ = obj;
          }
        }
      }
    }
  }

  if (obj_deadend_.id == 0)
  {
    ROS_WARN("Deadend: Free to go");
    return false;
  }
  else
  {
    ROS_WARN("Deadend: true");
    return true;
  }
}

// Check wheter obj is located before light/turn/goal within specified threshold
std::tuple<bool, bool, bool> DynamicPlanning::isBeforeLTG(obj_msgs::Obj obj, std::tuple<double, double, double> thresholds)
{
  double light_thres = std::get<0>(thresholds);
  double turn_thres = std::get<1>(thresholds);
  double goal_thres = std::get<2>(thresholds);
  bool before_light = false;
  bool before_turn = false;
  bool before_goal = false;

  for(int i = obj.index; i < global_trajectory_.waypoints.size(); i++)
  {
    std::string link_id = global_trajectory_.waypoints[i].CLID;

    // Before light
    if (!before_light)
    {
      for(int i = 0; i < traffic_list_.size(); i++)
      {
        if (traffic_list_[i].lat_offset <= HALF_LANE)
        {
          double diff = traffic_list_[i].s - obj.s;
          if (diff <= light_thres && diff > 0)
          {
            printf("BEFORE: %f m before light\n", diff);
            ROS_WARN("BEFORE: %f m before light", diff);
            before_light = true;
          }
          else
          {
            before_light = false;
          }
        }
      }
    }

    // Before turn
    if (!before_turn)
    {
      if (links_.at(link_id).road_type == RType::TURN)
      {
        double diff = sx_.m_x[i] - sx_.m_x[obj.index];
        if (diff <= turn_thres)
        {
          printf("BEFORE: %f m before turn\n", diff);
          ROS_WARN("BEFORE: %f m before turn", diff);
          before_turn = true;
        }
        else
        {
          before_turn = false;
        }
      }
    }

    // Before goal
    // Before light
    if (!before_goal)
    {
      double diff = goal_.s - obj.s;
      if (diff <= goal_thres && diff > 0)
      {
        printf("BEFORE: %f m before goal\n", diff);
        ROS_WARN("BEFORE: %f m before goal", diff);
        before_goal = true;
      }
      else
      {
        before_goal = false;
      }
    }
  }
  return std::make_tuple(before_light, before_turn, before_goal);
}

bool DynamicPlanning::isAdaptiveCruiseControlFinish()
{
  if (state_ == DynamicPlanning::STATE::CRUISING)
  {
    bool is_veh = isVehicle(headway_obj_);
    if (is_veh)
    {
      if (headway_obj_.type == OType::STATIC)
      {
        double curr_time = ros::Time::now().toSec();
        if (ego_speed_ == 0.0)
        {
          if (acc_timer_ == 0.0)
          {
            acc_timer_ += ros::Time::now().toSec() - curr_time;
          }
          else
          {
            acc_timer_ += curr_time - prev_acc_time_;
          }
        }
        else
        {
          acc_timer_ = 0.0;
        }
        printf("ACC waiting time: %f\n", acc_timer_);
        ROS_WARN("ACC waiting time: %f", acc_timer_);
        prev_acc_time_ = curr_time;
      }
      else
      {
        acc_timer_ = 0.0;
      }
    }
    else
    {
      acc_timer_ = 0.0;
      // 비정형의 경우 ACC waiting time을 무시하고 바로 RELAYING으로 넘어감
      if (headway_obj_.type == OType::ATYPICAL)
      {
        look_further_ = true;
        return true;
      }
      else
      {
        look_further_ = false;
        return false;
      }
    }

    // check avoidance time
    if (acc_timer_ >= avoidance_wait_time_thres_)
    {
      // is_avoid_ = true;
      look_further_ = true;
      return true;
    }
    else
    {
      look_further_ = false;
      // is_avoid_ = false;
      return false;
    }
  }
  else
  {
    // look_further_ = false;
    acc_timer_ = 0.0;
    return false;
  }
}

bool DynamicPlanning::isVehicle(obj_msgs::Obj obj)
{
  return (obj.type != OType::RED && obj.type!=OType::GOAL && obj.type!=OType::STOP && obj.type!=OType::LANES && obj.id != 0);
}

bool DynamicPlanning::isAtLaneChange(int index)
{
  return (global_trajectory_.waypoints[index].CLID.empty());
}

std::string DynamicPlanning::searchNextLink(int index)
{
  std::string link_id = global_trajectory_.waypoints[index].CLID;
  std::string llink_id = links_.at(link_id).LLID[0];
  std::string rlink_id = links_.at(link_id).RLID[0];

  // 물체 위치로 부터 30 m 뒤까지만 next link를 체크한다
  for(int i = index; i < global_trajectory_.waypoints.size(); i++)
  {
    std::string searched = global_trajectory_.waypoints[i].CLID;
    if (searched != link_id && searched != llink_id && searched != rlink_id)
    {
      return searched;
    }
  }
  return "";
}

int DynamicPlanning::searchTrajIndex(obj_msgs::Obj obj, path_msgs::Trajectory traj, double locate_range)
{
  double x = obj.pose.position.x;
  double y = obj.pose.position.y;
  int min_idx = 0;
  double min_dist = 99999;
  int start_idx = 0;
  int end_idx = 0;

  // Search 100 m ahead
  if (ego_.index - 200 >= 0)
  {
    start_idx = ego_.index - 200;
  }
  else
  {
    start_idx = 0;
  }

  if (ego_.index + 200 >= global_trajectory_.waypoints.size())
  {
    end_idx = global_trajectory_.waypoints.size() - 1;
  }
  else
  {
    end_idx = ego_.index + 200;
  }

  // printf("start_idx: %d, end_idx: %d, size: %d\n", start_idx, end_idx, global_trajectory_.waypoints.size());
  for (int i = start_idx; i <= end_idx; i++)
  {
    // printf("x: %f, y: %f\n", x, y);
    // printf("traj_x: %f, traj_y: %f\n", traj.waypoints[i].point.x, traj.waypoints[i].point.y);
    if (i < global_trajectory_.waypoints.size())
    {
      double dist = getDistance(x, y, traj.waypoints[i].point.x, traj.waypoints[i].point.y);
      // printf("id: %d, dist: %f\n", obj.id, dist);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_idx = i;
      }
    }
    else
    {
      break;
    }
  }
  if (min_dist < locate_range_) {
    // printf("obj id: %d\n", obj.id);
    // printf("min_dist: %f, min_idx: %d\n", min_dist, min_idx);
    return min_idx;
  } else {
    // printf("obj id: %d\n", obj.id);
    // printf("min_dist: %f, min_idx: %d\n", min_dist, min_idx);
    return -1;
  }
}

int DynamicPlanning::searchLinkIndex(geometry_msgs::Point point, path_msgs::Link link)
{
  int min_idx = 0;
  double min_dist = getDistance(point.x, point.y, link.geometry[0].x - offsetX_, link.geometry[0].y - offsetY_);

  for (int i = 1; i < link.geometry.size(); i++){
    double dist = getDistance(point.x, point.y, link.geometry[i].x - offsetX_, link.geometry[i].y - offsetY_);
    if (dist < min_dist){
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

int DynamicPlanning::searchLinkIndexByLength(path_msgs::Link link, int from_idx, int to_idx, double refer_length)
{

    double length = 0;
    int index = 0;

    if (to_idx > from_idx){
        for(int i = from_idx; i < to_idx; i++){
            length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }
    else if(from_idx > to_idx){
        for(int i = from_idx; i > to_idx; i--){
            length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }

    return index;
}

double DynamicPlanning::calcLength(path_msgs::Link link, int from_idx, int to_idx)
{
  double length = 0;

  if (to_idx > from_idx){
      for(int i = from_idx; i < to_idx; i++){
          length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
      }
  }
  else if(from_idx > to_idx){
      for(int i = from_idx; i > to_idx; i--){
          length += getDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
      }
  }
  else{
      length = 0;
  }

  return length;
}

double DynamicPlanning::getAngleDiff(double first_angle, double second_angle)
{
  double diff = first_angle - second_angle;
  if(abs(diff) >= PI)
  {
    double sign = (diff >= 0 ? 1.0 : -1.0);
    diff = diff - (sign * 2 * PI);
  }
  return diff;
}

double DynamicPlanning::getDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
