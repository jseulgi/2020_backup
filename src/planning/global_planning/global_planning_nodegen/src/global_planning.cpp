#include "global_planning.h"


PLANNING::PLANNING()
: map_initialized_(false),
pose_initialized_(false)
{
  nh = ros::NodeHandle("~");
  nh.param<double>("/lane_change/weight", weight, 4);
  nh.param<double>("/lane_change/length_lane_changed", length_lane_changed, 50);
  nh.param<int>("/bezier/size", bezier_size, 60);
  nh.param<double>("/push/length",push_length, 12.0);
  nh.param<double>("/push/bezier_length",bezier_push_length,15);
  nh.param<double>("/weights/weight_lane",weight_lane,3.0);
  nh.param<double>("/weights/atypical_weight",atypical_weight,1.5);
  nh.param("/debug/num",debug_num,1);

  mission_flag = false;
  is_simul_on = false;
  sent_flag = false;
  PathArrayPub = nh.advertise<visualization_msgs::Marker>("/globalPath",1, true);
  TrajectoryPub = nh.advertise<Trajectory>("/global_trajectory", 1, true);
  rviz_current_pose_pub_ = nh.advertise<visualization_msgs::Marker>("/rviz_current_pose", 1);
  Global2MissionPub = nh.advertise<mission_msgs::global2mission_list>("/global2mission_pub",1);
  Global2MissionStaticPub = nh.advertise<mission_msgs::global2mission>("/global2mission_static",1);
  Start_debugPub = nh.advertise<visualization_msgs::Marker>("/debug_start",1,true);
  End_debugPub = nh.advertise<visualization_msgs::Marker>("/debug_end",1,true);
  debug1Pub = nh.advertise<visualization_msgs::Marker>("/debug1",1,true);
  debug2Pub = nh.advertise<visualization_msgs::Marker>("/debug2",1,true);
  debug3Pub = nh.advertise<visualization_msgs::Marker>("/debug3",1,true);
  PathArray2startPub = nh.advertise<visualization_msgs::Marker>("/global2Start",1,true);
  StaticObjLinkIdPub = nh.advertise<obj_msgs::ObjList>("/StaticObj_link_id",1);
  GoalTestPub = nh.advertise<visualization_msgs::Marker>("/Goal_testing_marker",1,true);
  // < Subscriber >
  static ros::Subscriber RvizPositionSub = nh.subscribe("/initialpose",1,&PLANNING::RvizNavStartCallback,this);
  static ros::Subscriber RvizEndSub = nh.subscribe("/move_base_simple/goal",1, &PLANNING::GlobalPathHandler,this);
  static ros::Subscriber map_info_sub = nh.subscribe("/map_info", 1, &PLANNING::mapInfoCallback, this);
  static ros::Subscriber gps_odom_sub = nh.subscribe("/gps_odom", 1, &PLANNING::currentOdomCallback, this);
  static ros::Subscriber globalObj_sub = nh.subscribe("/two_static_obstacles",1,&PLANNING::globalObjCallback, this);
  static ros::Subscriber missions_sub = nh.subscribe("/mission_list",1,&PLANNING::missionCallback,this);
  // static ros::Subscriber BaseLinkOffSet = nh.subscribe("/base_link_offset", &PLANNING::BaseLinkOffSetCallback, this);

}

  PLANNING::~PLANNING(){
}
void PLANNING::currentOdomCallback(const nav_msgs::Odometry gps_odom)
{
  VehiclePosition = gps_odom.pose.pose;
  visualizeCurrentPose();
  pose_initialized_ = true;
  is_simul_on = false;
  // // Send "base_link" transform to "map"
  // tf::Transform transform;
  // transform.setRotation(tf::Quaternion(VehiclePosition.orientation.x, VehiclePosition.orientation.y, VehiclePosition.orientation.z, VehiclePosition.orientation.w));
  // transform.setOrigin(tf::Vector3(VehiclePosition.position.x, VehiclePosition.position.y, VehiclePosition.position.z));
  // tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

void PLANNING::mapInfoCallback(const path_msgs::Map map)
{
  std::vector<path_msgs::Link> links_msg = map.links;
  std::vector<path_msgs::Lane> lanes_msg = map.lanes;
  std::vector<path_msgs::Node> nodes_msg = map.nodes;
  OffsetMapX = map.OffsetMapX;
  OffsetMapY = map.OffsetMapY;

  for(auto it=links_msg.begin(); it != links_msg.end(); it++)
  {
    links.insert(make_pair(it->id, *it));
  }
  for(auto it=lanes_msg.begin(); it != lanes_msg.end(); it++)
  {
    lanes.insert(make_pair(it->id, *it));
  }
  for(auto it=nodes_msg.begin(); it != nodes_msg.end(); it++)
  {
    nodes.insert(make_pair(it->id, *it));
  }
  map_initialized_ = true;
}

void PLANNING::globalObjCallback(const obj_msgs::ObjList msg)
{
  std::cout << "Object Callback Running " << std::endl;
  geometry_msgs::Pose tmp_point;
  std::vector<geometry_msgs::Pose> tmp_points; // 물체의 각 변의 중심점
  std::vector<geometry_msgs::Pose> center_obj;
  std::vector<double> vector_dist1, vector_dist2;
  int cnt;

  for(int i=0; i<msg.objlist.size(); i++)
  {
    tmp_point.position.x = (msg.objlist[i].ul.x + msg.objlist[i].ur.x) / 2.0;
    tmp_point.position.y = (msg.objlist[i].ul.y + msg.objlist[i].ur.y) / 2.0;
    tmp_points.push_back(tmp_point);
    tmp_point.position.x =(msg.objlist[i].ul.x + msg.objlist[i].bl.x) / 2.0;
    tmp_point.position.y =(msg.objlist[i].ul.y + msg.objlist[i].bl.y) / 2.0;
    tmp_points.push_back(tmp_point);
    tmp_point.position.x =(msg.objlist[i].bl.x + msg.objlist[i].br.x) / 2.0;
    tmp_point.position.y =(msg.objlist[i].bl.y + msg.objlist[i].br.y) / 2.0;
    tmp_points.push_back(tmp_point);
    tmp_point.position.x =(msg.objlist[i].ur.x + msg.objlist[i].br.x) / 2.0;
    tmp_point.position.y =(msg.objlist[i].ur.y + msg.objlist[i].br.y) / 2.0;
    tmp_points.push_back(tmp_point);

    tmp_point.position.x = (msg.objlist[i].ul.x + msg.objlist[i].ur.x + msg.objlist[i].bl.x + msg.objlist[i].br.x) / 4.0;
    tmp_point.position.y = (msg.objlist[i].ul.y + msg.objlist[i].ur.y + msg.objlist[i].bl.y + msg.objlist[i].br.y) / 4.0;
    center_obj.push_back(tmp_point);
  }
  // 총 8개의 포인트가 tmp_points에 쌓임 물체 1번 - 0,1,2,3 :: 물체 2번 -4,5,6,7
  // 총 두개의 포인트가 center_obj에 쌓임 물체 1번 - 0 :: 물체 2번 - 1
  for(int i=0; i< center_obj.size(); i++)
  {
    for(int j=0; j<4; j++)
    {
      cnt = i * 4 + j;
      if(i==0)
      {
        vector_dist1.push_back(GetDistance(tmp_points[cnt].position.x, tmp_points[cnt].position.y, center_obj[i].position.x, center_obj[i].position.y));
      }
      else if(i==1)
      {
        vector_dist2.push_back(GetDistance(tmp_points[cnt].position.x, tmp_points[cnt].position.y, center_obj[i].position.x, center_obj[i].position.y));
      }
    }
  }

  int obj1_maxIndex1, obj1_maxIndex2, obj2_maxIndex1, obj2_maxIndex2; // obj1-> 0~3, obj2 -> 0~3

  obj1_maxIndex1 = std::max_element(vector_dist1.begin(), vector_dist1.end()) - vector_dist1.begin();
  vector_dist1.erase(std::max_element(vector_dist1.begin(), vector_dist1.end()));
  obj1_maxIndex2 = std::max_element(vector_dist1.begin(), vector_dist1.end()) - vector_dist1.begin();

  obj2_maxIndex1 = std::max_element(vector_dist2.begin(), vector_dist2.end()) - vector_dist2.begin();
  vector_dist2.erase(std::max_element(vector_dist2.begin(), vector_dist2.end()));
  obj2_maxIndex2 = std::max_element(vector_dist2.begin(), vector_dist2.end()) - vector_dist2.begin();

  // Calculate roll pitch yaw;
  double yaw1,yaw2;
  double roll1 = 0;
  double roll2 = 0;
  double pitch1 = 0;
  double pitch2 = 0;
  double dX1, dX2, dY1, dY2;
  dX1 = center_obj[0].position.x - tmp_points[obj1_maxIndex1].position.x;
  dY1 = center_obj[0].position.y - tmp_points[obj1_maxIndex1].position.y;
  dX2 = center_obj[1].position.x - tmp_points[obj2_maxIndex1].position.x;
  dY2 = center_obj[1].position.y - tmp_points[obj2_maxIndex1].position.y;
  yaw1 = atan2(dY1, dX1);
  yaw2 = atan2(dY2, dX2);
  tf2::Quaternion q1_rot, q2_rot;
  q1_rot.setRPY(roll1, pitch1, yaw1);
  q2_rot.setRPY(roll2, pitch2, yaw2);
  q1_rot.normalize();
  q2_rot.normalize();
  tf2::convert(q1_rot, center_obj[0].orientation);
  tf2::convert(q1_rot, tmp_points[obj1_maxIndex1].orientation);
  tf2::convert(q1_rot, tmp_points[obj1_maxIndex2].orientation);
  tf2::convert(q2_rot, center_obj[1].orientation);
  tf2::convert(q2_rot, tmp_points[obj2_maxIndex1].orientation);
  tf2::convert(q2_rot, tmp_points[obj2_maxIndex2].orientation);
  //

  vector_globalObj_pose.push_back(center_obj[0]);
  vector_globalObj_pose.push_back(tmp_points[obj1_maxIndex1]);
  vector_globalObj_pose.push_back(tmp_points[obj1_maxIndex2]);
  vector_globalObj_pose.push_back(center_obj[1]);
  vector_globalObj_pose.push_back(tmp_points[obj2_maxIndex1]);
  vector_globalObj_pose.push_back(tmp_points[obj2_maxIndex2]);

  std::cout << "obj1 " << std::endl;
  std::cout << center_obj[0] << std::endl;
  std::cout << "obj2 " << std::endl;
  std::cout << center_obj[1] << std::endl;

 pair<string, geometry_msgs::Point> obj1_for_pub, obj2_for_pub;
 center_obj[0].position.x = center_obj[0].position.x - OffsetMapX;
 center_obj[0].position.y = center_obj[0].position.y - OffsetMapY;
 center_obj[1].position.x = center_obj[1].position.x - OffsetMapX;
 center_obj[1].position.y = center_obj[1].position.y - OffsetMapY;
 obj1_for_pub = FindClosestLink(center_obj[0]);
 obj2_for_pub = FindClosestLink(center_obj[1]);

 StaticObj_list.objlist.clear();
 StaticObj.link_id = obj1_for_pub.first;
 StaticObj_list.objlist.push_back(StaticObj);
 StaticObj.link_id = obj2_for_pub.first;
 StaticObj_list.objlist.push_back(StaticObj);
 std::cout << "TEST: " << StaticObj_list.objlist[0] << std::endl;

 StaticObjLinkIdPub.publish(StaticObj_list);

  // FIND Closest LINK
  for(int i=0; i<vector_globalObj_pose.size(); i++)
  {
    vector_globalObj_pose[i].position.x = vector_globalObj_pose[i].position.x - OffsetMapX;
    vector_globalObj_pose[i].position.y = vector_globalObj_pose[i].position.y - OffsetMapY;
    std::cout << vector_globalObj_pose[i] << std::endl;
    pair<string, geometry_msgs::Point> tmp_pair;
    tmp_pair = FindClosestLink(vector_globalObj_pose[i]);
    vector_globalObj_link.push_back(tmp_pair);
  }

  // global 장애물에 해당하는 링크들 찾음 -> 찾은 링크들 OpenList에 안들어가게 해주면 됨 (if goal_link와 같지 않을때 )
  for(int i=0; i<vector_globalObj_link.size(); i++)
  {
    vector_globalObj_link_str.push_back(vector_globalObj_link[i].first);
  }
  // 찾아진 Link 중복 제거
  sort(vector_globalObj_link_str.begin(),vector_globalObj_link_str.end());
  vector_globalObj_link_str.erase(unique(vector_globalObj_link_str.begin(),vector_globalObj_link_str.end()),vector_globalObj_link_str.end());
  // 제거 완료 Open_list 에 vector_globalObj_link_str에 해당하는 것은 안들어가게 만들기

  std::cout << "static Obj Callback Done " <<std::endl;
}

void PLANNING::missionCallback(const mission_msgs::mission_list mission_msg)
{
  std::cout << "RUN MISSION CALLBACK " << std::endl;
  visualization_msgs::Marker StartPoint,EndPoint, debug1, debug2, debug3; // debug1,2 found closest point, debug3 vehicle pose
  // Set Marker
  StartPoint.header.frame_id = "map";
  StartPoint.header.stamp = ros::Time::now();
  StartPoint.action = visualization_msgs::Marker::ADD;
  StartPoint.pose.orientation.w = 1.0;
  StartPoint.id = 10;
  StartPoint.type = visualization_msgs::Marker::CUBE;
  StartPoint.scale.x = 5.0;
  StartPoint.scale.y = 5.0;
  StartPoint.scale.z = 5.0;
  StartPoint.color.g = 0.0;
  StartPoint.color.b = 1.0;
  StartPoint.color.r = 0.0;
  StartPoint.color.a = 1.0;

  EndPoint.header.frame_id = "map";
  EndPoint.header.stamp = ros::Time::now();
  EndPoint.action = visualization_msgs::Marker::ADD;
  EndPoint.pose.orientation.w = 1.0;
  EndPoint.id = 10;
  EndPoint.type = visualization_msgs::Marker::CUBE;
  EndPoint.scale.x = 5.0;
  EndPoint.scale.y = 5.0;
  EndPoint.scale.z = 5.0;
  EndPoint.color.g = 0.0;
  EndPoint.color.b = 1.0;
  EndPoint.color.r = 0.0;
  EndPoint.color.a = 1.0;

  debug1.header.frame_id = "map";
  debug1.header.stamp = ros::Time::now();
  debug1.action = visualization_msgs::Marker::ADD;
  debug1.pose.orientation.w = 1.0;
  debug1.id = 10;
  debug1.type = visualization_msgs::Marker::CUBE;
  debug1.scale.x = 5.0;
  debug1.scale.y = 5.0;
  debug1.scale.z = 5.0;
  debug1.color.g = 0.0;
  debug1.color.b = 0.0;
  debug1.color.r = 1.0;
  debug1.color.a = 1.0;

  debug2.header.frame_id = "map";
  debug2.header.stamp = ros::Time::now();
  debug2.action = visualization_msgs::Marker::ADD;
  debug2.pose.orientation.w = 1.0;
  debug2.id = 10;
  debug2.type = visualization_msgs::Marker::CUBE;
  debug2.scale.x = 5.0;
  debug2.scale.y = 5.0;
  debug2.scale.z = 5.0;
  debug2.color.g = 0.0;
  debug2.color.b = 0.0;
  debug2.color.r = 1.0;
  debug2.color.a = 1.0;

  debug3.header.frame_id = "map";
  debug3.header.stamp = ros::Time::now();
  debug3.action = visualization_msgs::Marker::ADD;
  debug3.pose.orientation.w = 1.0;
  debug3.id = 10;
  debug3.type = visualization_msgs::Marker::CUBE;
  debug3.scale.x = 5.0;
  debug3.scale.y = 5.0;
  debug3.scale.z = 5.0;
  debug3.color.g = 1.0;
  debug3.color.b = 0.0;
  debug3.color.r = 0.0;
  debug3.color.a = 1.0;

  // DONE

  // Clear Global Variable
  vector_closest_start.clear();
  vector_closest_end.clear();

  // DONE

  pair<int, int> tmp;
  for(int i=0; i<mission_msg.mission_lists.size(); i++)
  {
    vector_mission.push_back(mission_msg.mission_lists[i]);
  }
  std::cout << "Reading callback msg is done" << std::endl;
  // 승차 위치 -> 하차 위치 Planning (항상 고정) mission_flag변수를 이용해서 한번만 실행
  tf2::Quaternion q;
  q.setRPY(1, 0, 0);
  q.normalize();
  if(mission_flag == false)
  {
    std::cout << "Starting Find Global Paths" << std::endl;
    for(int i=0; i<mission_msg.mission_lists.size(); i++)
    {
      // Set Start & Goal
      geometry_msgs::Pose goal,start;
      goal.position = mission_msg.mission_lists[i].end_pos.pose.position;
      goal.orientation = mission_msg.mission_lists[i].end_pos.pose.orientation;
      goal.position.x = goal.position.x - OffsetMapX;
      goal.position.y = goal.position.y - OffsetMapY;
      start.position = mission_msg.mission_lists[i].start_pos.pose.position;
      start.orientation = mission_msg.mission_lists[i].end_pos.pose.orientation;
      start.position.x = start.position.x - OffsetMapX;
      start.position.y = start.position.y - OffsetMapY;
      tf2::convert(q, goal.orientation);
      tf2::convert(q, start.orientation);
      vector_start.push_back(start);
      vector_end.push_back(goal);
      // DONE

      CopyGoalPointfromRviz2(goal);
      ClosestStartLink = FindClosestLink(start);
      start_node_id = links.at(ClosestStartLink.first).from_node;
      ClosestEndLink = FindClosestLink(RvizGoal);
      vector_closest_start.push_back(ClosestStartLink);
      vector_closest_end.push_back(ClosestEndLink);
      goal_node_id = links.at(ClosestEndLink.first).to_node;
      GenerateGlobalPath(); // trajectory.waypoints.clear() 실행
      GenerateGoalPoint();
      correctWaypoints(trajectory); // global path trajectory 실행 완료
      tmp.first = mission_msg.mission_lists[i].mission_id;
      tmp.second = trajectory.waypoints.size();
      vector_calculated_missions.push_back(tmp);
      vector_trajectory.push_back(trajectory);
    }
    mission_flag = true;
  }
  std::cout << "SIZE: " << vector_calculated_missions.size() << std::endl;

  DrawingPath_debug1(vector_trajectory[debug_num]);
  StartPoint.pose = vector_start[debug_num];
  EndPoint.pose = vector_end[debug_num];
  Start_debugPub.publish(StartPoint);
  End_debugPub.publish(EndPoint);
  debug1.pose.position.x = vector_closest_start[debug_num].second.x;
  debug1.pose.position.y = vector_closest_start[debug_num].second.y;
  debug1.pose.position.z = 0;
  debug2.pose.position.x = vector_closest_end[debug_num].second.x;
  debug2.pose.position.y = vector_closest_end[debug_num].second.y;
  debug2.pose.position.z = 0;
  tf2::convert(q,debug1.pose.orientation);
  tf2::convert(q,debug2.pose.orientation);
  debug1Pub.publish(debug1);
  debug2Pub.publish(debug2);

  // 차량의 현재 위치부터 승차 위치까지는 Callback이 들어올때마다 진행해야함
  for(int i=0; i<mission_msg.mission_lists.size(); i++)
  {
    geometry_msgs::Pose start,goal; // Start = Vehicle Pose, Goal = Start_pos
    start = mission_msg.curr_pos.pose.pose;
    goal = mission_msg.mission_lists[i].start_pos.pose;
    start.position.x = start.position.x - OffsetMapX;
    start.position.y = start.position.y - OffsetMapY;
    goal.position.x = goal.position.x - OffsetMapX;
    goal.position.y = goal.position.y - OffsetMapY;
    tf2::convert(q,start.orientation);
    tf2::convert(q,goal.orientation);
    CopyGoalPointfromRviz2(goal);
    ClosestStartLink = FindClosestLink(start);
    vector_closest_start_vehicle.push_back(ClosestStartLink);
    start_node_id = links.at(ClosestStartLink.first).from_node;
    ClosestEndLink = FindClosestLink(RvizGoal);

    // Testing

    first_closest_link = ClosestStartLink;
    if(ClosestStartLink.first == ClosestEndLink.first)
    {
      //Find Index of start point and end point
      int index_starting, index_ending;
      index_starting = SearchClosestIndex(ClosestStartLink.second, links.at(ClosestStartLink.first));
      index_ending = SearchClosestIndex(ClosestEndLink.second, links.at(ClosestEndLink.first));
      if(index_starting < index_ending)
      {
        start_goal_flag = 0;
      }
      else
      {
        start_goal_flag = 1;
      }
    }
    else
    {
      start_goal_flag = 2;
    }

    std::cout << "flag: " << start_goal_flag << std::endl;

    // Case별로 시작 노드 끝 노드 달라야함
    if(start_goal_flag == 1)
    {
      start_node_id = links.at(ClosestStartLink.first).to_node;
      goal_node_id = links.at(ClosestEndLink.first).to_node;
    }
    else
    {
      start_node_id = links.at(ClosestStartLink.first).from_node;
      goal_node_id = links.at(ClosestEndLink.first).to_node;
    }

    goal_node_id = links.at(ClosestEndLink.first).to_node;
    GenerateGlobalPath();
    GenerateGoalPoint();
    correctWaypoints(trajectory);
    tmp.first = mission_msg.mission_lists[i].mission_id;
    tmp.second = trajectory.waypoints.size();
    vector_calculated_missions2start.push_back(tmp);
    vector_trajectory2start.push_back(trajectory);
  }

  std::cout << "VEHICLE2START SIZE: " << vector_calculated_missions2start.size() << std::endl;
  debug3.pose.position.x = vector_closest_start_vehicle[debug_num].second.x;
  debug3.pose.position.y = vector_closest_start_vehicle[debug_num].second.y;
  debug3.pose.position.z = 0;
  tf2::convert(q,debug3.pose.orientation);
  debug3Pub.publish(debug3);
  DrawingPath_debug2(vector_trajectory2start[debug_num]);

  for(int i=0; i< mission_msg.mission_lists.size(); i++)
  {
    pair<int, int> tmp;
    tmp.first = i;
    tmp.second = vector_calculated_missions2start[i].second + vector_calculated_missions[i].second;
    vector_calculated_missions_final.push_back(tmp);
  }

  global2mission_msg_list.lists.clear();

  for(int i=0; i<vector_calculated_missions_final.size(); i++)
  {
    std::cout << "ID: " << vector_calculated_missions_final[i].first << std::endl;
    std::cout << "SIZE: " << vector_calculated_missions_final[i].second << std::endl;
    global2mission_msg.id = vector_calculated_missions_final[i].first;
    global2mission_msg.num_point = vector_calculated_missions_final[i].second;
    global2mission_msg_list.lists.push_back(global2mission_msg);
  }

  std::cout << "SIZE: " << global2mission_msg_list.lists.size() << std::endl;

  Global2MissionPub.publish(global2mission_msg_list);

}

void PLANNING::GenerateGlobalPath(){
  ROS_INFO("Generating Global Path");

	Node current_node;
	Node goal_node;
	Link current_link;
	Cost current_cost;

  bool is_complete_path = false;

  current_cost = InitializeCost();

  closed_list.clear();
  open_list.clear();
  // trajectory_points.clear();
  trajectory.waypoints.clear();

  closed_list.push_back(current_cost);
  current_node = nodes.at( start_node_id );
  goal_node = nodes.at( goal_node_id );

  cout << "[START NODE] " << endl << current_node << endl;
  cout << "[GOAL NODE]" << endl << goal_node << endl;

  while( !is_complete_path ){
  	if( current_node.NLIDS[0] != "" ){
  		for( int i = 0; i < current_node.NLIDS.size(); i ++ ){
        // cout << i << " - Current node: " << current_node.NLIDS[i] << endl;
  			current_link = links.at( current_node.NLIDS[i] );
        // cout << i << " - Current link: " << current_link.id << endl;
			  Cost temp_cost = CalcCost( current_node, current_link, goal_node );

        UpdateOpenList(temp_cost);

  			if( current_link.RLID[0] != "" && links.at(current_link.RLID[0]).length > 35  ){
  				for ( int k = 0; k < current_link.RLID.size(); k ++ ){
  					Link right_link = links.at( current_link.RLID[k] );
  					Cost temp_cost = CalcCost(current_node, current_link, right_link, goal_node);
            UpdateOpenList(temp_cost);
            // cout << "NEXT RIGHT LINK : " << current_link.RLID[k] << endl;
  				}
  			}

  			if( current_link.LLID[0] != "" && links.at(current_link.LLID[0]).length > 35 ){
  				for (int k = 0; k < current_link.LLID.size(); k ++ ){
  					Link left_link = links.at( current_link.LLID[k] );
  					Cost temp_cost = CalcCost(current_node, current_link, left_link, goal_node);
            UpdateOpenList(temp_cost);
            // cout << "NEXT LEFT LINK : " << current_link.LLID[k] << endl;
  				}
  			}
  		}
  	}
  	else  {
		// cout << endl << "<---- Current node does not have any neighbors!!!! ---->" << endl;  // cout << "current node id : " << current_node.id << endl << endl;
	}

	// cout << "<Closed list>" << endl;
	// for( int k = 0; k < closed_list.size(); k ++ )
	// 	printf( "%s ", closed_list[k].node.c_str() );
    //
	// cout<< endl << "<Open list>" << endl;
	// for(int k = 0; k < open_list.size(); k ++ )
	// 	printf( "%s ", open_list[k].node.c_str() );
	// cout<< "----------------------------------------" << endl << endl;

	float min_score = 999999;
	unsigned int path_candidate = 0;

	for( int i = 0; i < open_list.size(); i ++ ){
		if( open_list[i].f_score < min_score ){
			min_score = open_list[i].f_score;
			// cout << "min score :  " << min_score << " id : " << open_list[i].node << endl;
			path_candidate = i;
		}
		// cout << i <<" : " <<  "min score :  " << min_score << " id : " << open_list[path_candidate].node << endl;
	}
      // cout << "min score :  " << min_score << " id : " << open_list[path_candidate].node << endl;

	closed_list.push_back( open_list[path_candidate] );
	open_list.erase( open_list.begin() + path_candidate );

	current_node = nodes.at( closed_list.back().node );

	if( closed_list.back().node == goal_node.id )
	    is_complete_path = true;
  }

  GenerateWayPoint( closed_list );
}

void PLANNING::UpdateOpenList(Cost temp_cost){

    bool is_new_open_list = false;
    bool is_there_global_obj = false;

    for( int j = 0; j < closed_list.size(); j ++ ){
        if(temp_cost.node == closed_list[j].node && temp_cost.parent_node == closed_list[j].parent_node){
            is_new_open_list = false;
            // cout << "This node is already in the closed list!!! -> " << temp_cost.node << " & " << closed_list[j].node << " " << temp_cost.parent_node <<" & " << closed_list[j].parent_node <<endl << endl;
            break;
        }
        else
            is_new_open_list = true;
    }

    if( is_new_open_list ){
        open_list.push_back(temp_cost);

        for( int j = 0; j < open_list.size() - 1; j ++ ){
            if(temp_cost.node == open_list[j].node && temp_cost.parent_node == open_list[j].parent_node ){
                // cout << endl << "There is same node in the open list!" << endl;
                if( temp_cost.g_score < open_list[j].g_score ){
                open_list[j].g_score = temp_cost.g_score;
                open_list[j].f_score = temp_cost.f_score;
                open_list[j].parent_node = temp_cost.parent_node;
                open_list[j].link = temp_cost.link;
                }
                open_list.pop_back();
            }
        }
        for(int l=0; l < vector_globalObj_link_str.size(); l++)
        {
          if(temp_cost.link == vector_globalObj_link_str[l] && ClosestEndLink.first != vector_globalObj_link_str[l])
          {
            open_list.pop_back();
          }
        }
    }
}

void PLANNING::GenerateWayPoint(vector<Cost> closed_list){

    trajectory_node.clear();
    trajectory_link.clear();
    string parent_id;

    for( int i=closed_list.size()-1; i > 0; i-- ){
        if(i == closed_list.size()-1){
            trajectory_node.push_back(closed_list[i].node);
            parent_id = closed_list[i].parent_node;
            trajectory_link.push_back(closed_list[i].link);
            // cout << closed_list[i].node_id << "  " << closed_list[i].parent_node_id <<endl;
        }
        else{
            if(parent_id == closed_list[i].node){
                trajectory_node.push_back(closed_list[i].node);
                parent_id = closed_list[i].parent_node;
                trajectory_link.push_back(closed_list[i].link);
            }
        }
    }

    trajectory_node.push_back(parent_id);

    reverse(trajectory_node.begin(), trajectory_node.end());
    reverse(trajectory_link.begin(), trajectory_link.end());

    cout << "[Global Path Node]: " << endl;
    for(int i=0; i<trajectory_node.size(); i++) cout<< trajectory_node[i] << endl;

    cout << "[Global Path Link]: " << endl;
    for(int i=0; i<trajectory_link.size(); i++) cout<< trajectory_link[i] << endl;

// Testing 2020 / 10 / 22
// std::cout << "TESTING: " <<  (trajectory_link[trajectory_link.size()-2]) << std::endl; // Goal 이전 링크
// std::cout << "SIZE: " << trajectory_link.size() << std::endl;
// std::cout << "TESTING2: " << trajectory_node[trajectory_node.size()-2] << std::endl; // NODE
// std::cout << "SIZE: " << trajectory_node.size() << std::endl;
int find_index;
bool is_done_flag = false;
int check_changed_idx; // 변경되는 index 확인 후 이후 index의 link 변경해야함
int check_link_idx;
int check_node_idx;
std::cout << "TESTING 1" << std::endl;
for(int i=trajectory_link.size()-2; i>0; i--)
{
    std::string check_lane, check_type, goal_lane; // change_lane: 차선 파악, check_type: 차선 type 변경
    std::string link_id = trajectory_link[i];
    std::string goal_id = trajectory_link[trajectory_link.size()-1];
    int link_id_size = link_id.size();
    goal_lane = goal_id.substr(link_id_size - 4);
    goal_lane.pop_back();
    goal_lane.pop_back();
    check_lane = link_id.substr(link_id_size - 4);
    check_lane.pop_back();
    check_lane.pop_back();
    check_type = link_id.substr(link_id_size - 6);
    check_type.pop_back();
    check_type.pop_back();
    check_type.pop_back();
    check_type.pop_back();

    if(check_lane == goal_lane  || check_type == "LL") // 바로 직전이 좌회전이거나, 골 이전 링크가 골과 같은 차선일때
    {
        // std::cout << "CHECK IF" <<std::endl;
        break;
    }
    else // 차선 변경이 있는경우. 이전에 차선 변경되게 바꿔 줘야함.
    {
        // std::cout << "CHECK ELSE" << std::endl;
        Link tmp_link = links.at(trajectory_link[i]);
        if(tmp_link.RLID[0] == "")
        {
            // std::cout << "CHECK IF2" << std::endl;
            continue;
        }
        else
        {
            // std::cout << "CHECK ELSE2" << std::endl;
            if(is_done_flag == false)
            {
                // trajectory_link[i] = tmp_link.RLID[0];
                check_link_idx = i;
                check_node_idx = i+1;
                // trajectory_node[trajectory_node.size()-2] = links.at(tmp_link.RLID[0]).to_node;
                // trajectory_node[i+1] = links.at(tmp_link.RLID[0]).to_node;
                is_done_flag = true;
            }
            else
            {
                break;
            }

        }
    }
}
std::cout << "TESTING 2" << std::endl;
std::string find_next_link_id; // 다음 link의 id 찾기
std::string split_str_tmp;

if(is_done_flag ==true)
{
    Link tmp_change = links.at(trajectory_link[check_link_idx]);
    trajectory_link[check_link_idx] = tmp_change.RLID[0];
    std::cout << "CHECK link idx: " << check_link_idx << std::endl;
    std::cout << "TESTING 3" << std::endl;
    for(int i=check_link_idx; i<trajectory_link.size()-1; i++)
    {

        Link tmp_link = links.at(trajectory_link[i]);
        for(int j=0; j<tmp_link.NLIDS.size(); j++)
        {
            split_str_tmp = tmp_link.NLIDS[j].substr(8);
            split_str_tmp.pop_back();
            split_str_tmp.pop_back();
            split_str_tmp.pop_back();
            split_str_tmp.pop_back();
            if(split_str_tmp != "LL" && split_str_tmp != "RR")
            {
                find_next_link_id = tmp_link.NLIDS[j];
            }
        }

        trajectory_link[i+1] = find_next_link_id;
        trajectory_node[i+1] = links.at(trajectory_link[i]).to_node;
    }
}



std::cout << "LINK SIZE: " << trajectory_link.size() << std::endl;
std::cout << "NODE SIZE: " << trajectory_node.size() << std::endl;

std::cout << "------------ TRAJECTORY LINK -------------" << std::endl;
for(int i=0; i<trajectory_link.size(); i++)
{
    std::cout << trajectory_link[i] << std::endl;
}

std::cout << "------------ TRAJECTORY NODE -------------" << std::endl;
for(int i=0; i<trajectory_node.size(); i++)
{
    std::cout << trajectory_node[i] << std::endl;
}


// trajectory_node[trajectory_node.size()-2] = links.at(trajectory_link.back()).from_node;


// std::cout << "------------- TRAJECTORY -------" << std::endl;
// trajectory_node.clear();
// for(int i=0; i<trajectory_link.size(); i++)
// {
//     if(i != trajectory_link.size()-1)
//     {
//         trajectory_node.push_back(links.at(trajectory_link[i]).from_node);
//     }
//     else
//     {
//         trajectory_node.push_back(links.at(trajectory_link[i]).to_node);
//     }
//     std::cout << trajectory_link[i] << std::endl;
// }



// Change Node

// std::string str_goal_link_id;
// std::string tmp_link_id, tmp_next_id;
// std::string split_tmp_link_id, split_goal_id; // 차선 정보 파악, 몇 차선인지?
// std::string split_goal_id_type; // LL 파악
// int link_id_size; // string split할때 이용
// int tmp_idx = trajectory_link.size()-2; // Index to check prev links from the Goal Link 이전 링크 부터 시작해서 감소..
// tmp_link_id = trajectory_link[tmp_idx]; // index가 변화되면서 link id 변화
// link_id_size = tmp_link_id.size(); // string Size
// int is_test_done = 0;
//
// // String split
// split_tmp_link_id = tmp_link_id.substr(link_id_size-4);
// split_tmp_link_id.pop_back();
// split_tmp_link_id.pop_back();
// split_goal_id = ClosestEndLink.first;
// split_goal_id = split_goal_id.substr(link_id_size-4);
// split_goal_id.pop_back();
// split_goal_id.pop_back();
// split_goal_id_type = (ClosestEndLink.first).substr(link_id_size-6);
// split_goal_id_type.pop_back();
// split_goal_id_type.pop_back();
// split_goal_id_type.pop_back();
// split_goal_id_type.pop_back();
//
// if(split_tmp_link_id != split_goal_id &&)


/////////// 10/22

/**
    // Goal Point의 이전 링크가 존재하는데, trajectory_link에서 골링크와 이전 링크의 차선이 다른경우, 이전 차선에서 차선 변경하게 바꿔야함.
    // 이전 차선의 right link가 있으면 link id를 right link의 것으로 변경
    // 이전 차선에 right link가 없으면 한개 더 전의 trajectory_link를
    std::string tmp_string;
    std::string tmp_link_id;
    std::string tmp_next_id;
    std::string must_change_lane_id;
    std::string split_tmp_link_id, split_goal_id;
    std::string split_goal_id_type;
    int link_id_size;
    int tmp_idx = trajectory_link.size()-2; // Index to check prev links from the GOAL Link. It decreases
    tmp_link_id = trajectory_link[tmp_idx]; // 위의 index가 바뀌면서 체크
    link_id_size = tmp_link_id.size(); // 링크의 string size
    // split_tmp_link_id = tmp_link_id.substr(link_id_size-4);
    // split_tmp_link_id.pop_back();
    // split_tmp_link_id.pop_back(); // 차선의 정보 파악
    split_goal_id = ClosestEndLink.first;
    split_goal_id = split_goal_id.substr(link_id_size-4);
    split_goal_id.pop_back();
    split_goal_id.pop_back(); // Goal Link의 차선 정보 파악
    split_goal_id_type = (ClosestEndLink.first).substr(link_id_size-6);
    split_goal_id_type.pop_back();
    split_goal_id_type.pop_back();
    split_goal_id_type.pop_back();
    split_goal_id_type.pop_back();
    bool is_test_done=false;
    int is_test_done_int = 0; // 0:없음 1: Left Link 존재  2: Right Link 존재 3: Left Right Link 모두 존재

    // std::cout << "Prev ID: " << tmp_link_id << std::endl;
    // std::cout << "LEFT LINK SIZE: " << links.at(tmp_link_id).LLID.size() << std::endl;
    // std::cout << "LEFT LINK: " << links.at(tmp_link_id).LLID[0] << std::endl;
    // std::cout << "RIGHT LINK SIZE: " << links.at(tmp_link_id).RLID.size() << std::endl;
    // std::cout << "GOAl LEFT LINK SIZE: " << links.at(ClosestEndLink.first).LLID.size() << std::endl;
    // Goal Link가 3차선이고 이전 링크가 2차선인경우도 고려 추가해야
    // while(split_tmp_link_id != split_goal_id)
    // {

    if(split_tmp_link_id != split_goal_id && split_goal_id_type != "RR" && split_goal_id_type != "LL" )
    {
        for(int i=trajectory_link.size()-2; i>0; i--)
        {
            tmp_link_id = trajectory_link[i];
            split_tmp_link_id = tmp_link_id.substr(link_id_size-4);
            split_tmp_link_id.pop_back();
            split_tmp_link_id.pop_back();
            if(links.at(tmp_link_id).LLID[0] != "")
            {
                std::cout << "LEFT LINK FOUND" << std::endl;
                is_test_done = true;
                is_test_done_int = 1; // LEFT Link 존재
            }
            if(links.at(tmp_link_id).RLID[0] != "")
            {
                // std::cout << "RIGHT LINK FOUND" << std::endl;
                is_test_done = true;
                if(is_test_done_int == 1)
                {
                    is_test_done_int = 3; // LEFT RIGHT LINK 존재
                }
                else
                {
                    is_test_done_int = 2; // RIGHT LINK 존재
                }
            }

            if(is_test_done_int != 0)
            {
                tmp_idx = i;
                // std::cout << "RESULT!!!!; " << trajectory_link[i] << std::endl;
                // std::cout << "INT: " << is_test_done_int << std::endl;
                // std::cout << "tmp_idx: " << trajectory_link.size() - tmp_idx << std::endl;
                // std::cout << "is_test_done_int: " << is_test_done_int << std::endl;
                // std::cout << "LL: " << links.at(trajectory_link[i]).LLID[0] << std::endl;
                // std::cout << "RR: " << links.at(trajectory_link[i]).RLID[0] << std::endl;
                // std::cout << "NODE: " << trajectory_node[i] << std::endl;
                if(is_test_done_int ==1)
                {
                    // std::cout << "is_test_done = 1: " << links.at(trajectory_link[i]).LLID[0] << std::endl;
                    trajectory_link[i] = links.at(trajectory_link[i]).LLID[0];
                }
                else if(is_test_done_int == 2)
                {
                    // std::cout << "is_test_done = 2: " << links.at(trajectory_link[i]).RLID[0] << std::endl;
                    trajectory_link[i] = links.at(trajectory_link[i]).RLID[0];

                    for(int j = i+1; j<trajectory_link.size(); j++)
                    {
                        for(int a=0; a<links.at(trajectory_link[j-1]).NLIDS.size(); a++)
                        {
                            std::string tmptmp;
                            tmptmp = links.at(trajectory_link[j-1]).NLIDS[a];
                            tmp_string = tmptmp.substr(link_id_size-4);
                            tmp_string.pop_back();
                            tmp_string.pop_back();
                            // std::cout << "tmp string: " << tmp_string << std::endl;
                            // std::cout << "size: " << links.at(trajectory_link[j-1]).NLIDS.size() << std::endl;
                            if(tmp_string == "02")
                            {
                                tmp_next_id = links.at(trajectory_link[j-1]).NLIDS[a];
                                trajectory_link[j] = tmp_next_id;
                            }
                        }
                        // trajectory_link[j] = tmp_next_id;
                        // std::cout << "Changed Trajectory: " << trajectory_link[j] << std::endl;

                    }
                }
                else
                {
                    std::cout << "Both LEFT RIGHT EXIST" << std::endl;
                }

                break;
            }
            is_test_done_int = 0;
        }

        std::cout << "NEW TRAJECTORY " << std::endl;
        for(int i=0; i<trajectory_link.size(); i++) cout<< trajectory_link[i] << endl;

        trajectory_node.clear();
        // Create New Trajectory NODE //
        std::cout << "NEW TRAJECTORY NODE" << std::endl;
        int test_dd = 0;
        for(int i=0; i<trajectory_link.size(); i++)
        {
            // if(test_dd == 1 && trajectory_node.back() == links.at(trajectory_link[i]).from_node)
            // {
            //     trajectory_node.pop_back();
            // }
            // else if(test_dd ==1 && trajectory_node.back() != links.at(trajectory_link[i]).from_node)
            // {
            //     trajectory_node.erase(trajectory_node.end()-1);
            // }
            if(test_dd == 1)
            {
                if(trajectory_node.back() != links.at(trajectory_link[i]).from_node)
                {
                    trajectory_node.push_back(links.at(trajectory_link[i]).to_node);
                }
                else
                {
                    trajectory_node.pop_back();
                    trajectory_node.push_back(links.at(trajectory_link[i]).from_node);
                    trajectory_node.push_back(links.at(trajectory_link[i]).to_node);
                }
            }
            else
            {
                trajectory_node.push_back(links.at(trajectory_link[i]).from_node);
                trajectory_node.push_back(links.at(trajectory_link[i]).to_node);
                test_dd = 1;
            }
        }
        for(int i=0; i<trajectory_node.size(); i++) std::cout << trajectory_node[i] << std::endl;
    }


**/



    ///////////////////////////////////////////////////////////

    Node waypoint;
    Link waypoint_link;
    Link waypoint_near_link;
    int matched_idx;
    geometry_msgs::Point temp;
    Link end_link = links.at(trajectory_link[trajectory_node.size()-2]);

    for(int i=0; i<trajectory_node.size()-1 ; i++){
        waypoint = nodes.at(trajectory_node[i]);
        waypoint_link = links.at(trajectory_link[i]);

        for(int j=0; j<waypoint.NLIDS.size(); j++){
            if(waypoint.NLIDS[j] == trajectory_link[i]){
                is_lane_changed = false;
                matched_idx = j;
                // ROS_INFO("No lane change");
                break;
            }
            else{
                is_lane_changed = true;

                if(waypoint.NLIDS[j] == waypoint_link.LLID[0] && waypoint.NLIDS[j] != ""){
                    waypoint_near_link = links.at(waypoint_link.LLID[0]);
                }
                else if(waypoint.NLIDS[j] == waypoint_link.RLID[0] && waypoint.NLIDS[j] != ""){
                    waypoint_near_link = links.at(waypoint_link.RLID[0]);
                }
            }
        }

        if(i==0){
            GenerateStartPoint(is_lane_changed);
        }

        if (i < trajectory_node.size()-2 && i > 0 ){
        // if (i < trajectory_node.size()-1 && i > 0 ){
            if(is_lane_changed) {
                // vector<geometry_msgs::Point> curve_points = ChangeLane(waypoint_near_link, waypoint_link);
                Trajectory curved_trajectory = ChangeLaneTrajectory(waypoint_near_link, waypoint_link);
                // for (int j=0; j<curve_points.size(); j++){
                //     temp.x = curve_points[j].x - OffsetMapX;
                //     temp.y = curve_points[j].y - OffsetMapY;
                //     trajectory_points.push_back(temp);
                // }

                for (int j=0; j<curved_trajectory.waypoints.size(); j++){
                    Waypoint wp = curved_trajectory.waypoints[j];
                    wp.point.x -= OffsetMapX;
                    wp.point.y -= OffsetMapY;
                    trajectory.waypoints.push_back(wp);
                }
            }
            else{
                for(int j=0; j<waypoint_link.geometry.size(); j++){
                    Waypoint wp;
                    wp.index = j;
                    wp.LLID = waypoint_link.LLID[0];
                    wp.RLID = waypoint_link.RLID[0];
                    wp.CLID = waypoint.NLIDS[matched_idx];
                    wp.point.x = waypoint_link.geometry[j].x - OffsetMapX;
                    wp.point.y = waypoint_link.geometry[j].y - OffsetMapY;
                    wp.speed = waypoint_link.speed;
                    trajectory.waypoints.push_back(wp);
                    // temp.x = waypoint_link.geometry[j].x - OffsetMapX;
                    // temp.y = waypoint_link.geometry[j].y - OffsetMapY;
                    // trajectory_points.push_back(temp);
                }
            }
        }
        // cout << "is lane chanege? " << is_lane_changed << " node: " << trajectory_node[i] << endl;
    }

}

void PLANNING::DrawingPath(){
    PathArray.header.frame_id = "map";
    PathArray.header.stamp = ros::Time::now();
    PathArray.action = visualization_msgs::Marker::ADD;
    PathArray.pose.orientation.w = 1.0;
    PathArray.id = 1;
    PathArray.type = visualization_msgs::Marker::LINE_STRIP;
    PathArray.scale.x = 0.8;
    PathArray.color.g = 1.0;
    PathArray.color.b = 0.5;
    PathArray.color.a = 1.0;

    PathArray.points.clear();
    for(int i=0; i<trajectory.waypoints.size(); i++)
        PathArray.points.push_back(trajectory.waypoints[i].point);
    PathArrayPub.publish(PathArray);

}

void PLANNING::DrawingPath_debug1(Trajectory traj)
{
  PathArray.header.frame_id = "map";
  PathArray.header.stamp = ros::Time::now();
  PathArray.action = visualization_msgs::Marker::ADD;
  PathArray.pose.orientation.w = 1.0;
  PathArray.id = 1;
  PathArray.type = visualization_msgs::Marker::LINE_STRIP;
  PathArray.scale.x = 0.8;
  PathArray.color.g = 1.0;
  PathArray.color.b = 0.5;
  PathArray.color.a = 1.0;

  PathArray.points.clear();
  for(int i=0; i<traj.waypoints.size(); i++)
  {
    PathArray.points.push_back(traj.waypoints[i].point);
  }
  PathArrayPub.publish(PathArray);
}

void PLANNING::DrawingPath_debug2(Trajectory traj)
{
  PathArray2start.header.frame_id = "map";
  PathArray2start.header.stamp = ros::Time::now();
  PathArray2start.action = visualization_msgs::Marker::ADD;
  PathArray2start.pose.orientation.w = 1.0;
  PathArray2start.id = 1;
  PathArray2start.type = visualization_msgs::Marker::LINE_STRIP;
  PathArray2start.scale.x = 0.8;
  PathArray2start.color.g = 0.0;
  PathArray2start.color.b = 1.0;
  PathArray2start.color.a = 1.0;

  PathArray2start.points.clear();
  for(int i=0; i<traj.waypoints.size(); i++)
  {
    PathArray2start.points.push_back(traj.waypoints[i].point);
  }
  PathArray2startPub.publish(PathArray2start);
}


Cost PLANNING::InitializeCost(){
	Cost current_cost;
	current_cost.node = start_node_id;
    current_cost.g_score = 0;
    current_cost.h_score = 0;
    current_cost.f_score = 0;
    current_cost.parent_node = "NONE";
    current_cost.link = "NONE";

    return current_cost;
}


Cost PLANNING::CalcCost(Node current_node, Link current_link, Node goal_node){

    // First Lane & Second LANE
    string cur_id = current_link.id;
    int idx = cur_id.size();
    string split_id = cur_id.substr(idx-4);
    split_id.pop_back();
    split_id.pop_back(); // 01, 02 ,03만 남게됨

    string first_lane = "01";
    string second_lane = "02";
    string third_lane = "03";



	Node next_node = nodes.at(current_link.to_node);
  float h_score;
	float g_score = current_link.length;

    if(split_id == second_lane || split_id == third_lane)
    {
        g_score = g_score * weight_lane;
        // std::cout << "Lane Weight is Multiplied " << std::endl;
    }

// 비정형 구간 Weight
  if(current_link.road_type == 6 || current_link.road_type == 8)
  {
    g_score = current_link.length * atypical_weight;
  }
  // if(GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) < 300)
  // {
  //     h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) / 1.0;
  // }
  // else if(GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) < 600)
  // {
  //     h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) / 2.0;
  // }
  // else if(GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) < 1500)
  // {
  //     h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) / 3.0;
  // }
  // else
  // {
  //     h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y) / 4.0;
  // }
	h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y);
	float f_score = g_score + h_score;

	// cout<< "parent node: [" << current_node.id << "] " << " currnet link : [" << current_link.id << "]" << " next node : [" << next_node.id << "]"<< endl;
	// cout << "g: " << g_score << " h: " << h_score << " f: " << f_score << endl << endl;

	Cost temp_cost;
	temp_cost.node = next_node.id;
	temp_cost.g_score = g_score;
	temp_cost.h_score = h_score;
	temp_cost.f_score = f_score;
	temp_cost.parent_node = current_node.id;
	temp_cost.link = current_link.id;

	return temp_cost;
}

Cost PLANNING::CalcCost(Node current_node, Link current_link, Link lrlink, Node goal_node){

	Node next_node = nodes.at( lrlink.to_node );

	float g_score = current_link.length * weight ; //GetDistance(next_node.geometry.x, next_node.geometry.y, current_node.geometry.x, current_node.geometry.y);
	float h_score = GetDistance(next_node.geometry.x, next_node.geometry.y, goal_node.geometry.x, goal_node.geometry.y);
	float f_score = g_score + h_score;

	// cout<< "parent node: [" << current_node.id << "] " << " currnet link : [" << lrlink.id << "]" << " next node : [" << next_node.id << "]"<< endl;
	// cout << "g: " << g_score << " h: " << h_score << " f: " << f_score << endl << endl;

	Cost temp_cost;
	temp_cost.node = next_node.id;
	temp_cost.g_score = g_score;
	temp_cost.h_score = h_score;
	temp_cost.f_score = f_score;
	temp_cost.parent_node = current_node.id;
	temp_cost.link = lrlink.id;

	return temp_cost;
}

float PLANNING::GetDistance(float pCurrent_x, float pCurrent_y, float pNext_x, float pNext_y){
	float Distance = sqrt(pow(pNext_x - pCurrent_x, 2) + pow(pNext_y - pCurrent_y, 2));
	return Distance;
}


void PLANNING::visualizeCurrentPose()
{
  visualization_msgs::Marker rviz_pose;

  rviz_pose.header.frame_id = "map";
  rviz_pose.header.stamp = ros::Time::now();
  rviz_pose.action = visualization_msgs::Marker::ADD;
  rviz_pose.pose = VehiclePosition;
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


/** This function is to use for the Vehicle with GPS */
// void PLANNING::GpsCallback(const sensor_msgs::NavSatFixConstPtr& fix){

//     double northing, easting;
//     string zone;
//     int flag=0;

//     if(fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX){
//         ROS_INFO("No Fix.");
//         return;
//     }

//     if(fix->header.stamp == ros::Time(0)){
//         return;
//     }

//     LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

//     CurrentPose.header = fix->header;

//     VehiclePosition.position.x = easting - OffsetMapX;
//     VehiclePosition.position.y = northing - OffsetMapY;
//     VehiclePosition.position.z = 0;

//     CurrentPose.pose.position.x = easting;
//     CurrentPose.pose.position.y = northing;
//     CurrentPose.pose.position.z = 0;
//     is_simul_on = false;
// }

// void PLANNING::GpsHeadingCallback(const ublox_msgs::NavPVTConstPtr& navheading){
//     NavheadingValueDeg = navheading->heading * 0.00001;
//     NavheadingValueDeg = -(NavheadingValueDeg - 90);
//     NavheadingValueRad = (NavheadingValueDeg * math_pi) / 180;

//     tf2::Quaternion quat;
//     geometry_msgs::Quaternion orientation;
//     quat.setRPY(0, 0, NavheadingValueRad);
//     quat.normalize();
//     orientation = tf2::toMsg(quat);
//     CurrentPose.pose.orientation = orientation;
//     VehPosePub.publish(CurrentPose);

// }

void PLANNING::GlobalPathHandler(const geometry_msgs::PoseStampedPtr& NavGoalMsg){

  clock_t start =  clock();

  //* 1. Conver ros message to geometry_msgs::Pose RvizGoal
  CopyGoalPointfromRviz(NavGoalMsg);

  // //* 2. Find the closest start link
  // if(!is_simul_on){
  //     ClosestStartLink = FindClosestLink(VehiclePosition);
  // }
  // else{
  //     ClosestStartLink = FindClosestLink(RvizPosition);
  // }
  // if (ClosestStartLink.first == ""){
  //   cout << "Cannot find closest start link" << endl;
  //   return;
  // }
  // else {
  //   cout << "Closest Start: " << ClosestStartLink.first << endl;
  // }
  // start_node_id = links.at(ClosestStartLink.first).from_node;

  // //* 3. Find the closest goal link
  // ClosestEndLink = FindClosestLink(RvizGoal);
  // if (ClosestEndLink.first == "") {
  //   cout << "Cannot find closest goal link" << endl;
  //   return;
  // }
  // else {
  //   cout << "Closest End: " << ClosestEndLink.first << endl;
  // }

  // goal_node_id = links.at(ClosestEndLink.first).to_node;


  // TESTING - 10/20
  if(!is_simul_on){
    ClosestStartLink = FindClosestLink(VehiclePosition);
  }
  else{
    ClosestStartLink = FindClosestLink(RvizPosition);
  }
  if(ClosestStartLink.first == ""){
    cout << "Cannot find closest start link" << endl;
    return;
  }
  else{
    cout << "Closest Start Link: " << ClosestStartLink.first << endl;
  }

  ClosestEndLink = FindClosestLink(RvizGoal);
  if(ClosestEndLink.first == ""){
    cout << "Cannot Find Closest goal link" << endl;
    return;
  }
  else{
    cout << "Closest End: " << ClosestEndLink.first << endl;
  }

// Marker
visualization_msgs::Marker goal_test_marker;
goal_test_marker.header.frame_id = "map";
goal_test_marker.header.stamp = ros::Time::now();
goal_test_marker.action = visualization_msgs::Marker::ADD;
goal_test_marker.pose.orientation.w = 1.0;
goal_test_marker.id = 10;
goal_test_marker.type = visualization_msgs::Marker::CUBE;
goal_test_marker.scale.x = 5.0;
goal_test_marker.scale.y = 5.0;
goal_test_marker.scale.z = 5.0;
goal_test_marker.color.g = 0.0;
goal_test_marker.color.b = 1.0;
goal_test_marker.color.r = 0.0;
goal_test_marker.color.a = 1.0;

goal_test_marker.pose.position = ClosestEndLink.second;
tf2::Quaternion q;
q.setRPY(1, 0, 0);
tf2::convert(q, goal_test_marker.pose.orientation);
GoalTestPub.publish(goal_test_marker);


  // Compare Start Link and End Link
  // 찾아진 ClosestStartLink 데이터를 first_closest_link에 넣어줌
  // 시작 위치와 끝 위치가 같은 링크일때 if에서 각각 index를 찾음
  // index 비교를 통해 start_goal_flag를 케이스에 맞춰 설정
  // 설정된 케이스에 맞춰 시작 노드와 끝 노드가 정해짐
  // Global 변수: start_goal_flag, first_closest_link 실행시 매번 업데이트됨.
  first_closest_link = ClosestStartLink;
  if(ClosestStartLink.first == ClosestEndLink.first)
  {
    //Find Index of start point and end point
    int index_starting, index_ending;
    index_starting = SearchClosestIndex(ClosestStartLink.second, links.at(ClosestStartLink.first));
    index_ending = SearchClosestIndex(ClosestEndLink.second, links.at(ClosestEndLink.first));
    std::cout << "HEREHEREHEREHEREHEREHEREHERE " << std::endl;
    std::cout << "START: " << index_starting << "    END: " << index_ending << std::endl;
    std::cout << "From: " << links.at(ClosestStartLink.first).from_node << " To: " << links.at(ClosestStartLink.first).to_node << std::endl;
    if(index_starting < index_ending)
    {
      start_goal_flag = 0;
    }
    else
    {
      start_goal_flag = 1;
    }
  }
  else
  {
    start_goal_flag = 2;
  }

  std::cout << "flag: " << start_goal_flag << std::endl;


  // Case별로 시작 노드 끝 노드 달라야함
  if(start_goal_flag == 1)
  {
    start_node_id = links.at(ClosestStartLink.first).to_node;
    goal_node_id = links.at(ClosestEndLink.first).to_node;
  }
  else
  {
    start_node_id = links.at(ClosestStartLink.first).from_node;
    goal_node_id = links.at(ClosestEndLink.first).to_node;
  }


  if (map_initialized_)
  {
    // cout << "start: " << ClosestStartLink.first << " end: " << ClosestEndLink.first << endl;
    //* 4. Generate global path
    GenerateGlobalPath();

    //* 5. Generate global path points
    GenerateGoalPoint();

    //* 6. Publish to local planning
    correctWaypoints(trajectory);
    // printf("Global trajectory size: %d\n", trajectory.waypoints.size());
    TrajectoryPub.publish(trajectory);

    //* 7. Draw out generated path
    DrawingPath();

    //* 7. Check operating time
    printf("%0.5f\n", (float)(clock() - start)/CLOCKS_PER_SEC);
  }
}

void PLANNING::CopyGoalPointfromRviz(const geometry_msgs::PoseStampedPtr& NavGoalMsg){

    RvizGoal.position.x = NavGoalMsg->pose.position.x;
    RvizGoal.position.y = NavGoalMsg->pose.position.y;
    RvizGoal.position.z = 0;

    RvizGoal.orientation.x = NavGoalMsg->pose.orientation.x;
    RvizGoal.orientation.y = NavGoalMsg->pose.orientation.y;
    RvizGoal.orientation.z = NavGoalMsg->pose.orientation.z;
    RvizGoal.orientation.w = NavGoalMsg->pose.orientation.w;

    ROS_INFO("Goal position x: %f y: %f", RvizGoal.position.x, RvizGoal.position.y);
}

void PLANNING::CopyGoalPointfromRviz2(const geometry_msgs::Pose NavGoalMsg)
{
  RvizGoal.position.x = NavGoalMsg.position.x;
  RvizGoal.position.y = NavGoalMsg.position.y;
  RvizGoal.position.z = 0;

  RvizGoal.orientation.x = NavGoalMsg.orientation.x;
  RvizGoal.orientation.y = NavGoalMsg.orientation.y;
  RvizGoal.orientation.z = NavGoalMsg.orientation.z;
  RvizGoal.orientation.w = NavGoalMsg.orientation.w;
}


void PLANNING::RvizNavStartCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& NavStartMsg){

    is_simul_on = true;
    RvizPosition.position.x = NavStartMsg->pose.pose.position.x;
    RvizPosition.position.y = NavStartMsg->pose.pose.position.y;
    RvizPosition.position.z = 0;

    RvizPosition.orientation.x = NavStartMsg->pose.pose.orientation.x;
    RvizPosition.orientation.y = NavStartMsg->pose.pose.orientation.y;
    RvizPosition.orientation.z = NavStartMsg->pose.pose.orientation.z;
    RvizPosition.orientation.w = NavStartMsg->pose.pose.orientation.w;

    ROS_INFO("Start position x: %f y: %f", RvizPosition.position.x, RvizPosition.position.y);
}


double PLANNING::RvizPose2yaw(geometry_msgs::Pose pose){
    double siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return yaw;
}

pair<string, geometry_msgs::Point> PLANNING::FindClosestLink(geometry_msgs::Pose CurPose){
    int count = 0;
    geometry_msgs::Point link_points, prev_link_points;
    pair<string, geometry_msgs::Point> selected_link;

    double dist, diff_theta;
    double yaw;
    double dist_threshold = 3;
    double slope, intersect, dist2link;
    double mindist2link = 1000.0;
    string minLinkId;

    // Global에서 받은 Start Pose를 yaw로 변환해야 함

    if(is_simul_on)
        yaw = RvizPose2yaw(RvizPosition);
    else
        yaw = tf::getYaw(CurPose.orientation);

    while(count < 1){
        count = 0;
        for(auto it=links.begin(); it!=links.end(); it++){
            // count = 0;
            // cout << "Link: " << it->second.id << endl;
            for(int i = 0; i < it->second.geometry.size(); i++){
                link_points.x = it->second.geometry[i].x - OffsetMapX;
                link_points.y = it->second.geometry[i].y - OffsetMapY;
                link_points.z = it->second.geometry[i].z;

                dist = GetDistance(CurPose.position.x, CurPose.position.y, link_points.x, link_points.y);
                // cout << "dist: " << dist << " id: " << it->second.id << " i: " << i <<endl;

                if (dist < dist_threshold){
                    diff_theta = yaw - atan2(link_points.y - CurPose.position.y, link_points.x - CurPose.position.x);

                    if(abs(diff_theta) >= math_pi){
                        double sign = GetSign(diff_theta);
                        diff_theta = diff_theta - (sign * 2 * math_pi);
                    }
                    // cout << "candidate" << it->second.id << " i: " << i << " yaw : " << NavheadingValueDeg <<" theta: " << diff_theta <<endl;
                    if(abs(diff_theta) < math_pi/2){

                        if(i == it->second.geometry.size()-1){
                            prev_link_points.x = it->second.geometry[i-1].x - OffsetMapX;
                            prev_link_points.y = it->second.geometry[i-1].y - OffsetMapY;
                            prev_link_points.z = it->second.geometry[i-1].z;
                            slope = GetSlope(prev_link_points, link_points);
                        }
                        else{
                            prev_link_points.x = it->second.geometry[i+1].x - OffsetMapX;
                            prev_link_points.y = it->second.geometry[i+1].y - OffsetMapY;
                            prev_link_points.z = it->second.geometry[i+1].z;
                            slope = GetSlope(link_points, prev_link_points);
                        }

                        intersect = GetIntersectY(slope, link_points);
                        dist2link = abs(slope * CurPose.position.x + (-1)*CurPose.position.y + intersect) / sqrt(pow(slope,2) + pow(1,2));
                        // cout << " candidate : " << it->second.id << " Distance : " << dist  << " slope: " << slope << " d2l: " << dist2link << endl;

                        if(dist2link < mindist2link){
                            count++;
                            mindist2link = dist2link;
                            selected_link.first = it->second.id;
                            selected_link.second = link_points;
                            // cout << " link : " << it->second.id << " Distance : " << dist  << " slope: " << slope << " d2l: " << dist2link << endl;
                            // cout<< "slected:  " << minLinkId << endl;
                        }
                    }
                }
            }
        }
        // cout << count << endl;
        dist_threshold += 0.5;
    }
    // std::cout << "Selected: " << selected_link.first << std::endl;

    ROS_INFO("Geometry points for searching: %d", GeometryPair.size());

    return selected_link;
}

double PLANNING::GetSign(double angle){
    if(angle >= 0.0)
        return 1.0;
    else
        return -1.0;
}

double PLANNING::GetSlope(geometry_msgs::Point a, geometry_msgs::Point b){
    double slope;
    return slope = (b.y - a.y) / (b.x - a.x);
}

double PLANNING::GetIntersectY(double slope, geometry_msgs::Point a){
    double intersect;
    return intersect = -(slope * a.x) + a.y;
}

void PLANNING::GenerateGoalPoint(){

    ROS_INFO("Generating end points of the path");

    Link goal_link = links.at(closed_list.back().link);
    int min_idx = 0;
    Link to_link;

    // vector<geometry_msgs::Point> generated_points;
    Trajectory generated_trajectory;
    geometry_msgs::Point temp;
    // std::cout << "DOES IT CHANGE LANE?? : " << is_lane_changed << std::endl;

    if(is_lane_changed){
        std::cout << "CHANGE LANE ! ! ! ! ! !" << std::endl;
        if(goal_link.LLID[0]!=""){
            to_link = links.at(links.at(ClosestEndLink.first).LLID[0]);
        }
        else if(goal_link.RLID[0]!=""){
            to_link = links.at(links.at(ClosestEndLink.first).RLID[0]);
        }
        // generated_points = GoalLinkChangeLane(to_link, goal_link);
        generated_trajectory = GoalLinkChangeLaneTrajectory(to_link, goal_link);

        for(int i=0; i < generated_trajectory.waypoints.size(); i++){
            // temp.x = generated_points[i].x - OffsetMapX;
            // temp.y = generated_points[i].y - OffsetMapY;
            Waypoint wp = generated_trajectory.waypoints[i];
            wp.point.x -= OffsetMapX;
            wp.point.y -= OffsetMapY;
            // trajectory_points.push_back(temp);
            trajectory.waypoints.push_back(wp);
        }
    }
    // 10m 밀어주기, Closeset Index 그냥 한번 더 찾음 (min_idx 이용 가능))
    else
    {
        double calcdist, accumdist;
        min_idx = SearchClosestIndex(ClosestEndLink.second, goal_link);
        int checkidx;
        bool is_within = false; // true -> 10m이상의 여유를 가짐

        for(int i=min_idx; i<goal_link.geometry.size()-1; i++)
        {
            calcdist = GetDistance(goal_link.geometry[i].x, goal_link.geometry[i].y, goal_link.geometry[i+1].x, goal_link.geometry[i+1].y);
            accumdist += calcdist;
            if(accumdist > push_length && is_within == false)
            {
                checkidx = i;
                is_within = true;
            }
        }
        // std::cout << "IDX: " << checkidx << std::endl;
        // std::cout << "Is Within: " << is_within << std::endl;

        if(is_within)
        {
            if(trajectory.waypoints[0].point.x != (goal_link.geometry[0].x - OffsetMapX))
            {
                for(int i=0; i<checkidx+1; i++)
                {
                    Waypoint wp;
                    wp.index = i;
                    wp.CLID = goal_link.id;
                    wp.RLID = goal_link.RLID[0];
                    wp.LLID = goal_link.LLID[0];
                    wp.point.x = goal_link.geometry[i].x - OffsetMapX;
                    wp.point.y = goal_link.geometry[i].y - OffsetMapY;
                    wp.speed = goal_link.speed;
                    trajectory.waypoints.push_back(wp);
                }
            }
            else
            {
                for(int i = min_idx; i<checkidx+1; i++)
                {
                    Waypoint wp;
                    wp.index = i;
                    wp.CLID = goal_link.id;
                    wp.RLID = goal_link.RLID[0];
                    wp.LLID = goal_link.LLID[0];
                    wp.point.x = goal_link.geometry[i].x - OffsetMapX;
                    wp.point.y = goal_link.geometry[i].y - OffsetMapY;
                    wp.speed = goal_link.speed;
                    trajectory.waypoints.push_back(wp);
                }
            }
        }
        else // Next Link 찾아야함 ROADTYPE 8,9는 회전 Link, 따라서 회전 링크의 경우 제외
        {
            std::vector<string> nextlink_vec;
            string nextlink;
            int closestidx;
            bool is_done=false;
            string str_find1 = "RR";
            string str_find2 = "LL";
            for(int i=0; i < links.at(ClosestEndLink.first).NLIDS.size(); i++)
            {
                // std::cout << links.at(ClosestEndLink.first).NLIDS[i].find(str_find1) << endl;
                if(links.at(ClosestEndLink.first).NLIDS[i].find(str_find1) == string::npos && links.at(ClosestEndLink.first).NLIDS[i].find(str_find2) == string::npos)
                {
                    nextlink = links.at(ClosestEndLink.first).NLIDS[i];
                    // std::cout << "NEXT: " << nextlink << std::endl;
                }
                // nextlink_vec.push_back(links.at(ClosestEndLink.first).NLIDS[i]); 좌회전 우회전 두개 같이 있는경우는 없고 거의 직진 및 우회전이 존재함
            }
            if(nextlink != "")
            {
                // 다음 Link의 geometry index중 10m가 넘어갈때index를 찾음
                for(int i=0; i<links.at(nextlink).geometry.size(); i++)
                {
                    calcdist = GetDistance(links.at(nextlink).geometry[i].x, links.at(nextlink).geometry[i].y, links.at(nextlink).geometry[i+1].x, links.at(nextlink).geometry[i+1].y);
                    accumdist += calcdist;
                    if(accumdist > push_length && !is_done)
                    {
                        is_done = true;
                        closestidx = i;
                    }
                }
                // 가까운 점 index찾았고,
                // Waypoint 선언 먼저 끝
                Waypoint wp;
                wp.RLID = links.at(nextlink).RLID[0];
                wp.LLID = links.at(nextlink).LLID[0];
                wp.speed = links.at(nextlink).speed;
                // 마지막 링크 전체 push_back + 다음 Link index까지 push_back
                // 같은 링크일때는 다음링크만 push_back
                if(trajectory.waypoints[0].point.x != (goal_link.geometry[0].x - OffsetMapX))
                {
                    wp.index = goal_link.geometry.size() + closestidx;
                    // Goal Link 전체 Pushback
                    for(int i=0; i<goal_link.geometry.size(); i++)
                    {
                        wp.CLID = goal_link.id;
                        wp.point.x = goal_link.geometry[i].x - OffsetMapX;
                        wp.point.y = goal_link.geometry[i].y - OffsetMapY;
                        trajectory.waypoints.push_back(wp);
                    }
                    for(int i=0; i<closestidx; i++)
                    {
                        wp.CLID = nextlink;
                        wp.point.x = links.at(nextlink).geometry[i].x - OffsetMapX;
                        wp.point.y = links.at(nextlink).geometry[i].y - OffsetMapY;
                        trajectory.waypoints.push_back(wp);
                    }
                }
                // // 같은 링크일떄
                else
                {
                    wp.index = goal_link.geometry.size() + closestidx - min_idx +1;
                    for(int i=min_idx; i<goal_link.geometry.size(); i++)
                    {
                        wp.CLID = goal_link.id;
                        wp.point.x = goal_link.geometry[i].x - OffsetMapX;
                        wp.point.y = goal_link.geometry[i].y - OffsetMapY;
                        trajectory.waypoints.push_back(wp);
                    }

                    for(int i=0; i<closestidx; i++)
                    {
                        wp.CLID = nextlink;
                        wp.point.x = links.at(nextlink).geometry[i].x - OffsetMapX;
                        wp.point.y = links.at(nextlink).geometry[i].y - OffsetMapY;
                        trajectory.waypoints.push_back(wp);
                    }

                }

            }


          }
        }
}

void PLANNING::GenerateStartPoint(bool is_lane_changed){

    ROS_INFO("Generating start points of the path");
    Link start_link = links.at(ClosestStartLink.first);
    Link to_link;
    int min_idx;
    vector<geometry_msgs::Point> generated_points;
    geometry_msgs::Point temp;

    //신경 안써도 되는부분
    if(is_lane_changed){
        if (start_link.LLID[0]!=""){
            to_link = links.at(links.at(ClosestStartLink.first).LLID[0]);
        }
        else if(start_link.RLID[0]!=""){
            to_link = links.at(links.at(ClosestStartLink.first).RLID[0]);
        }

        for(int i=0; i<to_link.geometry.size(); i++){
            // temp.x = to_link.geometry[i].x - OffsetMapX;
            // temp.y = to_link.geometry[i].y - OffsetMapY;
            // trajectory_points.push_back(temp);
            Waypoint wp;
            wp.index = i;
            wp.CLID = to_link.id;
            wp.RLID = to_link.RLID[0];
            wp.LLID = to_link.LLID[0];
            wp.point.x = to_link.geometry[i].x - OffsetMapX;
            wp.point.y = to_link.geometry[i].y - OffsetMapY;
            wp.speed = to_link.speed;
            trajectory.waypoints.push_back(wp);
        }
    }

    else{
        Link tmp = links.at(trajectory_link[0]);
        float mindist = 9999;
        float mindist_start = 9999;
        float calcdist = 0;
        float calcdist_start = 0;
        int closestidx_start = 0;
        int closestidx = 0;
        double accumdist = 0;
        std::cout << "LINK ID: " << tmp.id << std::endl; //start_goal_flag
        if(ClosestStartLink.first == ClosestEndLink.first){
            if(start_goal_flag != 1)
            {
                for(int i=0; i<tmp.geometry.size(); i++)
              {
                  calcdist = GetDistance(tmp.geometry[i].x, tmp.geometry[i].y, ClosestEndLink.second.x + OffsetMapX, ClosestEndLink.second.y + OffsetMapY);
                  calcdist_start = GetDistance(tmp.geometry[i].x, tmp.geometry[i].y, VehiclePosition.position.x + OffsetMapX, VehiclePosition.position.y + OffsetMapY);
                  if(calcdist < mindist)
                  {
                      closestidx = i;
                      mindist = calcdist;
                      std::cout << "it works" << std::endl;
                  }
              }

              std::cout << "START: " << closestidx << std::endl;
              for(int i=0; i<closestidx; i++)
              {
                Waypoint wp;
                wp.index = i;
                wp.CLID = start_link.id;
                wp.RLID = start_link.RLID[0];
                wp.LLID = start_link.LLID[0];
                wp.point.x = tmp.geometry[i].x - OffsetMapX;
                wp.point.y = tmp.geometry[i].y - OffsetMapY;
                wp.speed = tmp.speed;
                trajectory.waypoints.push_back(wp);
              }
            }
            else
            {
              // Find idx of Starting point
              Link temp_link = links.at(first_closest_link.first);
              mindist = 9999;
              for(int i=0; i<temp_link.geometry.size(); i++)
              {
                calcdist = GetDistance(temp_link.geometry[i].x, temp_link.geometry[i].y, first_closest_link.second.x + OffsetMapX, first_closest_link.second.y + OffsetMapY);
                if(calcdist < mindist)
                {
                  closestidx = i;
                  mindist = calcdist;
                }
              }

              Waypoint wp;
              std::cout << "IDX TEST: " << closestidx << std::endl;
              std::cout << "Link: " << first_closest_link.first << std::endl;

              for(int i=closestidx; i<links.at(first_closest_link.first).geometry.size(); i++)
              {
                wp.index = i;
                wp.CLID = start_link.id;
                wp.RLID = start_link.RLID[0];
                wp.LLID = start_link.LLID[0];
                wp.point.x = temp_link.geometry[i].x - OffsetMapX;
                wp.point.y = temp_link.geometry[i].y - OffsetMapY;
                wp.speed = temp_link.speed;
                trajectory.waypoints.push_back(wp);
              }


              for(int i=0; i<tmp.geometry.size(); i++)
              {
                wp.index = i;
                wp.CLID = start_link.id;
                wp.RLID = start_link.RLID[0];
                wp.LLID = start_link.LLID[0];
                wp.point.x = tmp.geometry[i].x - OffsetMapX;
                wp.point.y = tmp.geometry[i].y - OffsetMapY;
                wp.speed = tmp.speed;
                trajectory.waypoints.push_back(wp);
              }
            }
        }
        else
        {
            for(int i = 0; i < tmp.geometry.size() ; i++){
                // temp.x = start_link.geometry[i].x - OffsetMapX;
                // temp.y = start_link.geometry[i].y - OffsetMapY;
                // trajectory_points.push_back(temp);
                Waypoint wp;
                wp.index = i;
                wp.CLID = start_link.id;
                wp.RLID = start_link.RLID[0];
                wp.LLID = start_link.LLID[0];
                wp.point.x = tmp.geometry[i].x - OffsetMapX;
                wp.point.y = tmp.geometry[i].y - OffsetMapY;
                wp.speed = tmp.speed;
                trajectory.waypoints.push_back(wp);
            }
        }
    }

}

void PLANNING::correctWaypoints(Trajectory &traj)
{
  vector<Waypoint> corrected_waypoints;
  vector<Waypoint> orig_waypoints = traj.waypoints;
  for(int i=1; i < orig_waypoints.size(); i++)
  {
    Waypoint prev_wp = orig_waypoints[i-1];
    Waypoint wp = orig_waypoints[i];
    corrected_waypoints.push_back(prev_wp);
    if(GetDistance(wp.point.x, wp.point.y, prev_wp.point.x, prev_wp.point.y) < 0.05)
    {
      i++;
      continue;
    }
    if (i == orig_waypoints.size()-1)
    {
      corrected_waypoints.push_back(wp);
    }
  }
  traj.waypoints = corrected_waypoints;
}

Trajectory PLANNING::ChangeLaneTrajectory(Link from_link, Link to_link){

    cout << "-> Change Lane" << endl;

    double length_from_link;
    double length_to_link;
    double length_to_p1;
    double length_to_p2;
    double length_to_p3;
    double length_to_p4;

    int coef = 3;

    bool is_lane_long = false;

    Bezier bezier_param;
    geometry_msgs::Point P1, P2, P3, P4;

    Trajectory curved_trajectory;

    length_from_link = CalcLength(from_link, 0, from_link.geometry.size()-1);
    length_to_link = CalcLength(to_link, 0, to_link.geometry.size()-1);

    // ROS_INFO_STREAM("length_from_link " << length_from_link);
    // ROS_INFO_STREAM("length_to_link " << length_to_link);


    if(length_from_link > length_lane_changed || length_to_link > length_lane_changed){
        is_lane_long = true;
        length_to_p1 = (length_from_link - length_lane_changed) / 2 ;
        length_to_p2 = ((length_from_link - length_lane_changed) / 2 ) + (length_lane_changed / coef);
        length_to_p3 = ((length_to_link - length_lane_changed) / 2 ) + ((length_lane_changed * (coef-1))/ coef);
        length_to_p4 = ((length_to_link - length_lane_changed) / 2 ) + length_lane_changed;
    }
    else{
        length_to_p1 = 0;
        length_to_p2 = length_from_link / coef;
        length_to_p3 = (length_to_link * (coef-1))/ coef;
        length_to_p4 = length_to_link;
    }

    // ROS_INFO_STREAM(length_to_p1);
    // ROS_INFO_STREAM(length_to_p2);
    // ROS_INFO_STREAM(length_to_p3);
    // ROS_INFO_STREAM(length_to_p4);
    // ROS_INFO_STREAM(is_lane_long);

    bezier_param.p1_idx = SearchIndex(from_link, 0, from_link.geometry.size()-1, length_to_p1);
    bezier_param.p2_idx = SearchIndex(from_link, 0, from_link.geometry.size()-1, length_to_p2);
    bezier_param.p3_idx = SearchIndex(to_link, 0, to_link.geometry.size()-1, length_to_p3);
    bezier_param.p4_idx = SearchIndex(to_link, 0, to_link.geometry.size()-1, length_to_p4);

    P1 = from_link.geometry[bezier_param.p1_idx];
    P2 = from_link.geometry[bezier_param.p2_idx];
    P3 = to_link.geometry[bezier_param.p3_idx];
    P4 = to_link.geometry[bezier_param.p4_idx];

    // ROS_INFO_STREAM(P1.x << ", " << P1.y);
    // ROS_INFO_STREAM(P2.x << ", " << P2.y);
    // ROS_INFO_STREAM(P3.x << ", " << P3.y);
    // ROS_INFO_STREAM(P4.x << ", " << P4.y);


    if(is_lane_long){
        for(int i=0; i<bezier_param.p1_idx+1; i++){
            Waypoint wp;
            wp.point = from_link.geometry[i];
            wp.index = i;
            wp.CLID = from_link.id;
            wp.RLID = from_link.RLID[0];
            wp.LLID = from_link.LLID[0];
            wp.speed = from_link.speed;
            wp.type = 1;
            curved_trajectory.waypoints.push_back(wp);
        }
    }

    // double bezier_size = 20.0;

    for(int t=0; t < bezier_size; t++){
        double s = ((double) t)/bezier_size;
        Waypoint bezier_wp;
        bezier_wp.index = t;
        if (t < bezier_size / 2)
        {
          bezier_wp.CLID = from_link.id;
          bezier_wp.RLID = from_link.RLID[0];
          bezier_wp.LLID = from_link.LLID[0];
          bezier_wp.speed = from_link.speed;
        }
        else
        {
          bezier_wp.CLID = to_link.id;
          bezier_wp.RLID = to_link.RLID[0];
          bezier_wp.LLID = to_link.LLID[0];
          bezier_wp.speed = to_link.speed;
        }
        bezier_wp.point.x = P1.x * pow((1-s),3) + 3 * P2.x * s * pow(1-s, 2) + 3 * P3.x * pow(s, 2) * (1-s) + P4.x * pow(s, 3);
        bezier_wp.point.y = P1.y * pow((1-s),3) + 3 * P2.y * s * pow(1-s, 2) + 3 * P3.y * pow(s, 2) * (1-s) + P4.y * pow(s, 3);
        bezier_wp.type = 1;
        curved_trajectory.waypoints.push_back(bezier_wp);
    }

    if(is_lane_long){
        for(int i=bezier_param.p4_idx; i<to_link.geometry.size(); i++){
            Waypoint wp;
            wp.point = to_link.geometry[i];
            wp.index = i;
            wp.CLID = to_link.id;
            wp.RLID = to_link.RLID[0];
            wp.LLID = to_link.LLID[0];
            wp.speed = to_link.speed;
            curved_trajectory.waypoints.push_back(wp);
        }
    }

    return curved_trajectory;
}

Trajectory PLANNING::GoalLinkChangeLaneTrajectory(Link from_link, Link to_link){

    cout << "-> Change Lane" << endl;

    double length_from_link;  // The length of the from_link which is the previous link before changing the lane
    double length_to_link;    // The length of the to_link which is the future link after the changing the lane
    double length_to_p1;
    double length_to_p2;
    double length_to_p3;
    double length_to_p4;

    int coef = 3;

    bool is_lane_long = false;

    Bezier bezier_param;
    geometry_msgs::Point bezier;
    geometry_msgs::Point P1, P2, P3, P4;

    Trajectory curved_trajectory;

    int to_link_start_idx;
    int from_link_end_idx;


    bezier_param.start_idx = 0;
    // From Link에서 차선 변경 이전 링크
    from_link_end_idx = SearchClosestIndex(ClosestEndLink.second, from_link);

    to_link_start_idx = 0;
    // To Link에서 차선 변경 이후 링크
    bezier_param.end_idx = SearchClosestIndex(ClosestEndLink.second, to_link);

    length_from_link = CalcLength(from_link, bezier_param.start_idx, from_link_end_idx);
    length_to_link = CalcLength(to_link, to_link_start_idx, bezier_param.end_idx); // push 10m

    if(length_from_link > length_lane_changed || length_to_link > length_lane_changed){
        is_lane_long = true;
        length_to_p1 = (length_from_link - length_lane_changed)/2.0 ;
        length_to_p2 = ((length_from_link - length_lane_changed))/2.0 + (length_lane_changed / coef);
        length_to_p3 = ((length_to_link - length_lane_changed))/2.0 + ((length_lane_changed * (coef-1))/ coef);
        length_to_p4 = ((length_to_link - length_lane_changed))/2.0 + length_lane_changed;
        // std::cout << "TESTING1 ! ! !" <<std::endl;
        // std::cout << "p1: " << length_to_p1 <<std::endl;
        // std::cout << "p2: " << length_to_p2 <<std::endl;
        // std::cout << "p3: " << length_to_p3 <<std::endl;
        // std::cout << "p4: " << length_to_p4 <<std::endl;
    }
    else{
        length_to_p1 = 0;
        length_to_p2 = length_from_link / coef;
        length_to_p3 = (length_to_link * (coef-1))/ coef;
        length_to_p4 = length_to_link;
        // std::cout << "TESTING2 ! ! !" <<std::endl;
    }

    bezier_param.p1_idx = SearchIndex(from_link, bezier_param.start_idx, from_link_end_idx, length_to_p1);
    bezier_param.p2_idx = SearchIndex(from_link, bezier_param.start_idx, from_link_end_idx, length_to_p2);
    bezier_param.p3_idx = SearchIndex(to_link, to_link_start_idx, bezier_param.end_idx, length_to_p3);
    bezier_param.p4_idx = SearchIndex(to_link, to_link_start_idx, bezier_param.end_idx, length_to_p4);

    // std::cout << "TO LINK ID: " << to_link.id << std::endl;
    //
    // std::cout << "Length from_LINK: " << length_from_link << std::endl;
    // std::cout << "Length to LINK: " << length_to_link << std::endl;
    // std::cout << "length_lane_changed: " << length_lane_changed << std::endl;
    // std::cout << "IS LANE LONG????? : " << is_lane_long << std::endl;
    //
    // std::cout << "Length to p1: " << length_to_p1 << std::endl;
    // std::cout << "Length to p2: " << length_to_p2 << std::endl;
    // std::cout << "Length to p3: " << length_to_p3 << std::endl;
    // std::cout << "Length to p4: " << length_to_p4 << std::endl;
    //
    //
    // std::cout << "Searched Index " << std::endl;
    // std::cout << "P1: " << bezier_param.p1_idx << std::endl;
    // std::cout << "P2: " << bezier_param.p2_idx << std::endl;
    // std::cout << "P3: " << bezier_param.p3_idx << std::endl;
    // std::cout << "P4: " << bezier_param.p4_idx << std::endl;

    P1 = from_link.geometry[bezier_param.p1_idx];
    P2 = from_link.geometry[bezier_param.p2_idx];
    P3 = to_link.geometry[bezier_param.p3_idx];
    P4 = to_link.geometry[bezier_param.p4_idx];


    ROS_INFO_STREAM(P1.x-OffsetMapX << ", " << P1.y-OffsetMapY);
    ROS_INFO_STREAM(P2.x-OffsetMapX << ", " << P2.y-OffsetMapY);
    ROS_INFO_STREAM(P3.x-OffsetMapX << ", " << P3.y-OffsetMapY);
    ROS_INFO_STREAM(P4.x-OffsetMapX << ", " << P4.y-OffsetMapY);

    if(is_lane_long){
        for(int i=0; i<bezier_param.p1_idx + 1; i++){
            Waypoint wp;
            wp.index = i;
            wp.point.x = from_link.geometry[i].x;
            wp.point.y = from_link.geometry[i].y;
            wp.LLID = from_link.LLID[0];
            wp.RLID = from_link.RLID[0];
            wp.CLID = from_link.id;
            wp.speed = from_link.speed;
            wp.type = 1;
            curved_trajectory.waypoints.push_back(wp);
        }
    }

    for(int t=0; t < bezier_size; t++){
        double s = ((double) t)/bezier_size;
        Waypoint bezier_wp;
        bezier_wp.index = t;
        if (t < bezier_size / 2)
        {
          bezier_wp.CLID = from_link.id;
          bezier_wp.RLID = from_link.RLID[0];
          bezier_wp.LLID = from_link.LLID[0];
          bezier_wp.speed = from_link.speed;
        }
        else
        {
          bezier_wp.CLID = to_link.id;
          bezier_wp.RLID = to_link.RLID[0];
          bezier_wp.LLID = to_link.LLID[0];
          bezier_wp.speed = to_link.speed;
        }
        bezier_wp.point.x = P1.x * pow((1-s),3) + 3 * P2.x * s * pow(1-s, 2) + 3 * P3.x * pow(s, 2) * (1-s) + P4.x * pow(s, 3);
        bezier_wp.point.y = P1.y * pow((1-s),3) + 3 * P2.y * s * pow(1-s, 2) + 3 * P3.y * pow(s, 2) * (1-s) + P4.y * pow(s, 3);
        bezier_wp.type = 1;
        curved_trajectory.waypoints.push_back(bezier_wp);
    }

    Waypoint wp;
    wp.index = bezier_param.p4_idx;
    wp.point.x = ClosestEndLink.second.x + OffsetMapX;
    wp.point.y =  ClosestEndLink.second.y + OffsetMapY;
    wp.CLID = to_link.id;
    wp.RLID = to_link.RLID[0];
    wp.LLID = to_link.LLID[0];
    wp.speed = to_link.speed;
    ROS_INFO_STREAM("End link id: " << to_link.id);
    curved_trajectory.waypoints.push_back(wp);

    // std::cout << "X: " << curved_trajectory.waypoints.back().point.x - OffsetMapX << std::endl;
    // std::cout << "Y: " << curved_trajectory.waypoints.back().point.y - OffsetMapY << std::endl;

    /* New Process
    1. 마지막 Curved_trajectory_waypoints의 x,y를 이용하여 goal 링크의 끝점과 거리 계산
    2. 계산된 거리가 push_length보다 길면, goal 링크에서 목적지와 가까운 포인트를 찾고, 그 인덱스를 찾으며, push_length
    만큼 밀어낸 point의 인덱스를 찾음
    3. 계산된 거리가 push_length보다 짧으면, goal 링크의 Next Link를 가져와 push_length까지인 포인트를 찾고, 그 인덱스를 찾으며
    push_length만큼 밀어낸 point의 인덱스를 찾음
    4. 2번에 해당하는 경우 curved_trajectory.waypoints에 목적지와 가까운 포인트의 index부터 밀어낸 포인트의 index까지 추가
    5. 3번에 해당하는 경우 curved_trajectory.waypoints에 현재 링크의 끝 인덱스까지와 찾아낸 point의 인덱스까지 추가함
    */
    // geometry_msgs::Point tmpforidx;
    // tmpforidx.x = curved_trajectory.waypoints.back().point.x - OffsetMapX;
    // tmpforidx.y = curved_trajectory.waypoints.back().point.y - OffsetMapY;
    // int closest_idx;
    // closest_idx = SearchClosestIndex(tmpforidx, links.at(ClosestEndLink.first));
    // std::cout << "closest_idx: " << closest_idx << std::endl;
    // std::cout << "bezier_idx : " << bezier_param.p4_idx << std::endl;


    // Push 10 Meter for the ACC
    // to_link  = Goal LINK
    // Bezier_param.p4_ix = Goal Point index

    /* Process
        1. Check the distance btw goal point to end point of the link
        2. If it is over 10 meter, then find the index that 10 meter away from the goal point.
            - push_back goal_idx ~ found_idx to the curved_trajectory
        3. If it is less than 10 meter, find the next link which is the stratigh link.
        4. Find the Index in found next link that 10 meter away from the goal point
    */


    // Bezier의 마지막 포인트와 골 링크의 끝 점과의 거리 구하기
    // -> Bezier의 마지막 점이 아니라 위의 curved_trajectory.waypoints의 마지막 포인트를 이용해야함
    // double dist2endlink = CalcLength(to_link,bezier_param.p4_idx,to_link.geometry.size()-1);
    geometry_msgs::Point tmpforidx;
    tmpforidx.x = curved_trajectory.waypoints.back().point.x - OffsetMapX;
    tmpforidx.y = curved_trajectory.waypoints.back().point.y - OffsetMapY;
    int closest_idx;
    closest_idx = SearchClosestIndex(tmpforidx, links.at(ClosestEndLink.first));

    double dist2endlink = CalcLength(to_link,closest_idx,to_link.geometry.size()-1);
    int finalgoal_idx;
    Link goal_nextlink;
    string next;
    bool is_within = true;
    string str_find1 = "RR";
    string str_find2 = "LL";

    // 목적지에서 링크의 끝점까지 거리가 push_length보다 많이 남았을 때
    if(dist2endlink > bezier_push_length)
    {
        // push하는데까지의 index찾기
        finalgoal_idx = SearchIndex(to_link, closest_idx, to_link.geometry.size()-1, bezier_push_length);
        // std::cout << "Bezier Push Length: " << bezier_push_length << std::endl;
        // std::cout << "Dist2Length: " << dist2endlink << std::endl;
        // std::cout << "Within " << std::endl;
        // std::cout << "p4: " << bezier_param.p4_idx << std::endl;
        // std::cout << "final: " << finalgoal_idx << std::endl;
        // std::cout << "p4_x: " << to_link.geometry[bezier_param.p4_idx].x - OffsetMapX << std::endl;
        // std::cout << "p4_y: " << to_link.geometry[bezier_param.p4_idx].y - OffsetMapY << std::endl;
        // std::cout << "final_x: " << to_link.geometry[finalgoal_idx].x - OffsetMapX << std::endl;
        // std::cout << "final_y: " << to_link.geometry[finalgoal_idx].y - OffsetMapY << std::endl;
    }
    // 골 링크의 끝점까지 거리가 push_length보다 짧으면, 다음 링크를 찾고, 다음 링크의 index를 찾아야함
    else
    {
        for(auto i=0; i<links.at(to_link.id).NLIDS.size(); i++)
        {
            // Next Link의 id에 LL, RR이 없으면 next에 저장
            if(links.at(to_link.id).NLIDS[i].find(str_find1) == string::npos && links.at(to_link.id).NLIDS[i].find(str_find2) == string::npos)
            {
                next = links.at(to_link.id).NLIDS[i];
                // std::cout << "NOT Within" << std::endl;
                // std::cout << "NEXT: " << nextlink << std::endl;
            }
        }
        // 위에서 찾은 next를 이용하여 goal_nextlink찾고 이용
        goal_nextlink = links.at(next);
        // std::cout << "GOAL NEXTLINK: " << goal_nextlink.id << std::endl;
        // 다음 링크에서 index찾기
        finalgoal_idx = SearchIndex(goal_nextlink, 0, goal_nextlink.geometry.size()-1, bezier_push_length - dist2endlink);
        is_within = false;
    }

    // 위에서 if에 들어갔을 경우
    if(is_within)
    {
        // 베지에 마지막 포인트 부터 위에서 찾은 final goal idx까지 추가로 Waypoint쌓아줌
        for(auto i=closest_idx; i<finalgoal_idx; i++)
        {
          wp.index = i;
          wp.point.x = to_link.geometry[i].x;
          wp.point.y = to_link.geometry[i].y;
          wp.CLID = to_link.id;
          wp.RLID = to_link.RLID[0];
          wp.LLID = to_link.LLID[0];
          wp.speed = to_link.speed;
          curved_trajectory.waypoints.push_back(wp);
        }
    }
    // 위에서 else에 들어갔을 경우
    else
    {
        // 베지에 마지막 포인트 부터 링크의 끝점까지 추가
        for(auto i=bezier_param.p4_idx; i<to_link.geometry.size(); i++)
        {
          wp.index = i;
          wp.point.x = to_link.geometry[i].x;
          wp.point.y = to_link.geometry[i].y;
          wp.CLID = to_link.id;
          wp.RLID = to_link.RLID[0];
          wp.LLID = to_link.LLID[0];
          wp.speed = to_link.speed;
          curved_trajectory.waypoints.push_back(wp);
        }
        // 다음 링크의 시작점 부터 위에서 찾은 final goal idx까지 추가로 Waypoint 쌓아줌
        for(auto i=0; i<finalgoal_idx; i++)
        {
            wp.index = i + to_link.geometry.size();
            wp.point.x = goal_nextlink.geometry[i].x;
            wp.point.y = goal_nextlink.geometry[i].y;
            wp.CLID = goal_nextlink.id;
            wp.RLID = goal_nextlink.RLID[0];
            wp.LLID = goal_nextlink.LLID[0];
            wp.speed = goal_nextlink.speed;
            curved_trajectory.waypoints.push_back(wp);
        }
    }




    return curved_trajectory;
}

int PLANNING::SearchClosestIndex(geometry_msgs::Point point, Link link){
    double min_dist = 9999;
    int min_idx = 0;
    double dist;

    for(int i=0; i<link.geometry.size()-1; i++){
        dist = GetDistance(point.x, point.y, link.geometry[i].x - OffsetMapX, link.geometry[i].y - OffsetMapY);
        // cout<< "dist : " << dist << " min: " << min_dist << " idx : " << min_idx << endl;
        if(dist < min_dist){
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}

double PLANNING::CalcLength(Link link, int from_idx, int to_idx){

    double length = 0;
    if (to_idx > from_idx){
        for(int i = from_idx; i < to_idx; i++){
            length += GetDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
        }
    }
    else if(from_idx > to_idx){
        for(int i = from_idx; i > to_idx; i--){
            length += GetDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
        }
    }
    else{
        length = 0;
    }

    return length;
}


int PLANNING::SearchIndex(Link link, int from_idx, int to_idx, double refer_length){

    double length = 0;
    int index = 0;

    if (to_idx > from_idx){
        for(int i = from_idx; i < to_idx; i++){
            length += GetDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i+1].x, link.geometry[i+1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }
    else if(from_idx > to_idx){
        for(int i = from_idx; i > to_idx; i--){
            length += GetDistance(link.geometry[i].x, link.geometry[i].y, link.geometry[i-1].x, link.geometry[i-1].y);
            if(length >= refer_length){
                index = i;
                break;
            }
        }
    }

    return index;
}
