
#ifndef DYNAMIC_PLANNING_H
#define DYNAMIC_PLANNING_H

#include <ros/ros.h>
#include <vector>
#include <thread>
#include <mutex>
#include <map>
#include <cmath>
#include <queue>
#include <iostream>
#include <deque>
#include <functional>
#include <numeric>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "path_msgs/Node.h"
#include "path_msgs/Link.h"
#include "path_msgs/Cost.h"
#include "path_msgs/Bezier.h"
#include "path_msgs/Trajectory.h"
#include "path_msgs/Waypoint.h"
#include "path_msgs/Map.h"
#include "path_msgs/State.h"

#include "obj_msgs/ObjList.h"
#include "obj_msgs/Obj.h"
#include "obj_msgs/Ego.h"
// #include "object_tracker/Tracker.h"

#include "control_msgs/VehicleState.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/ColorRGBA.h"
#include "novatel_gps_msgs/NovatelXYZ.h"
#include "spline.h"
#include "timercpp.h"

#define PI 3.14159265359
#define HALF_LANE 1.5


enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4
};

enum class LControl
{
  KEEP = 0,
  LEFT = 1,
  RIGHT = 2,
  BOTH = 3
};

enum LCode
{
  CENTER = 1,
  UTURN = 2,
  LANE = 3,
  NO_LANE_CHANGE = 5,
  NO_PARKING = 8,
  NO_STOPPING = 9,
  BARRIER = 99
};

enum RType
{
  NORMAL = 1,
  ATYPICAL_STRAIGHT = 6,
  INTERSECTION = 7,
  ATYPICAL_TURN = 8,
  TURN = 9,
  BEFORE_TURN = 10
};

enum OType{
  NOINTEREST = 0,
  APPROACHING = 1,
  STATIC = 2,
  FRONTOBJ  = 3,
  OPPOSITE = 4,
  RED = 5,
  EGO = 6,
  GOAL = 7,
  STOP = 8,
  ATYPICAL = 9,
  LANES = 10,
  FAKE = 11
};


struct Candidate
{
  int index;
  double smc; // smoothness cost
  double coc; // comfortability cost
  double oc; // obstacle cost
  double gc; // global path following cost
  double lc; // lane cost
  double max_k; // maximum curvature
  tk::spline sp; // spline path
  geometry_msgs::Point lah_point;
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point collision_point;
  double time_to_collision;
  obj_msgs::Obj collision_obj;
};

class DynamicPlanning
{

public:

	typedef enum STATE
	{
		INITIALIZING = -1,
		RELAYING = 0,
		CRUISING = 1,
    BLOCKED = 2,
    HARD_BRAKE = 3,
	} State;

	DynamicPlanning();
	~DynamicPlanning();

  void run();


private:

  /* ros */
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber current_odom_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber obstacle_index_sub_;
	ros::Subscriber global_trajectory_sub_;
  ros::Subscriber object_info_sub_;
  ros::Subscriber traffic_info_sub_;
  ros::Subscriber map_info_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber ego_state_sub_;
  ros::Subscriber lookahead_sub_;
  ros::Subscriber atypical_obj_sub_;
  ros::Publisher ego_speed_pub_;
  ros::Publisher ego_acc_pub_;
  ros::Publisher ego_location_pub_;
  ros::Publisher lookahead_dist_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher road_state_pub_;
  std::vector<ros::Publisher> publishers_;
  ros::Rate *rate_;
  tf::TransformBroadcaster tfBroadcaster;

  /* rviz visualization*/
  ros::Publisher rviz_current_pose_pub_;
  ros::Publisher rviz_goal_point_pub_;
  ros::Publisher rviz_fake_obj_pub_;
  ros::Publisher rviz_ellipse_pub_;
  ros::Publisher rviz_stop_nodes_pub_;
  ros::Publisher rviz_velocity_pub_;
  ros::Publisher ref_path_pub_;
  visualization_msgs::MarkerArray ellipse_markers_;

  /* Car specification */
  double width_;
  double length_;
  double gps_to_hood_;
  double gps_to_rear_;

	/* params */
	int loop_rate_;
  bool is_rect_;
  bool is_ellipse_;
  /* topics and frame_ids */
  std::string gps_topic_;
  std::string map_frame_;

  /* classes */
	State state_;

  /* variables */
	geometry_msgs::PoseStamped current_pose_;
	path_msgs::Trajectory global_trajectory_;
  path_msgs::Trajectory local_trajectory_;

  /* spline */
  double s_start_;
  double s_end_;
  double curv_;
  tk::spline sx_;
  tk::spline sy_;
  double global_len_;

  /* speed limit */
  double lat_offset_;
  double sign_limit_;
  double stop_dist_;
  double lat_acc_limit_;
  double safety_gain_;
  double ref_speed_;

  /* path candidates */
  std::vector<path_msgs::Bezier> beziers_;
  int path_size_;
  double offset_;
  double dist_coeff_;
  double min_dist_;
  double max_dist_;
  double safety_distance_;

  std::vector<Candidate> candidates_;
  std::vector<tk::spline> sp_candidates_;
  std::vector<std::vector<std::tuple<double,double>>> cart_candidates_;
  Candidate prev_cand_;

  /* avoidance */
  bool first_sight_;
  double avoidance_distance_;
  double light_thres_;
  double turn_thres_;
  double goal_thres_;
  /* cost */
  std::vector<double> smoothness_cost_;
  std::vector<double> static_cost_;
  std::vector<double> dynamic_cost_;
  std::vector<double> R_all_;
  std::vector<double> R_lanes_;
  int opt_path_wo_obs_;
  int opt_path_;
  int prev_opt_path_;
  double ws_; // static cost weight
  double wc_; // comfortability cost weight
  double wl_; // lateral offset weight
  double a_; // smoothness cost weight
  double b_; // continuity cost weight

  /* Traffic sign */
  std::vector<obj_msgs::Obj> traffic_list_;

  /* Goal */
  obj_msgs::Obj goal_;
  double stop_push_;
  double red_push_;

  /* Ego */
  obj_msgs::Obj ego_;
  geometry_msgs::Point hood_pos_;
  geometry_msgs::Point rear_pos_;
  tf::Vector3 ego_vel_;
  int prev_index_;
  double ego_speed_;
  double ego_acc_;
  double yaw_;
  double relative_yaw_;
  double locate_range_;
  bool is_atypical_;

  /* Stop nodes */
  double prev_node_idx_;
  double prev_time_;
  double wait_time_thres_;
  std::vector<obj_msgs::Obj> all_stop_nodes_;
  std::vector<obj_msgs::Obj> stop_nodes_;
  std::vector<bool> is_nodes_;
  std::vector<double> timer_;

  /* Obstacle */
  obj_msgs::Obj headway_obj_;
  obj_msgs::Obj prev_collision_obj_;
  double prev_static_time_;
  double static_timer_;
  double avoidance_wait_time_thres_;
  double prev_acc_time_;
  double acc_timer_;
  std::vector<obj_msgs::Obj> veh_list_;
  std::vector<obj_msgs::Obj> raw_veh_list_;
  std::vector<obj_msgs::Obj> atypical_obj_list_;

  double enlarge_size_;
  double obs_thres_;
  double lane_thres_;
  double cut_in_thres_;
  double sigma_;
  double obs_cost_;
  bool switch_;
  bool is_blocked_;
  bool is_deadend_;
  bool is_wait_;
  bool is_brake_;
  bool is_avoid_;
  bool is_avoiding_;
  bool look_further_;
  bool obj_ahead_;
  obj_msgs::Obj obj_deadend_;

  /* ACC */
  bool acc_on_;
  double max_decel_;
  double dr_;
  double dr_dot_;
  double braking_distance_;
  double lookahead_distance_;
  int count_;
  obj_msgs::Obj prev_obj_;
  // int prev_obj_id_;

  /* Initialization boolean */
  bool pose_initialized_;
  bool node_initialized_;
  bool trajectory_initialized_;
  bool map_initialized_;
  bool dynamic_initialized_;
  bool path_following_initialized_;

  /* map */
  std::map <std::string, path_msgs::Link> links_;
  std::map <std::string, path_msgs::Node> nodes_;
  std::map <std::string, path_msgs::Lane> lanes_;
  double offsetX_;
  double offsetY_;

  /* threads */
  std::thread publish_thread_;
  std::mutex mutex_;
  bool terminate_thread_;


  /*** Functions ***/
  /* rviz */
  void visualizeStopNodes();
  void visualizeCurrentPose();
  void visualizeGoalPoint();
  void visualizePathCandidate(int optimal_index);
  void visualizeEllipse(obj_msgs::Obj obj);

	/* callbacks */
  void atypicalObjCallback(const obj_msgs::ObjList msg);
  void currentOdomCallback(const nav_msgs::Odometry msg);
  void trajectoryCallback(const path_msgs::Trajectory msg);
  void mapInfoCallback(const path_msgs::Map map_msg);
  void vehCallback(const obj_msgs::ObjList msg);
  void rawVehCallback(const obj_msgs::ObjList msg);
  void egoStateCallback(const control_msgs::VehicleState msg);
  void lookaheadCallback(const std_msgs::Float64 msg);
  void trafficCallback(const obj_msgs::ObjList msg);

  /* Control */
  void searchObjAhead();
  void updateObjAhead();
  void executeAdaptiveCruiseControl(obj_msgs::Obj obj);
  bool isAdaptiveCruiseControlFinish();
  void hardBrake();
  double getNominalDistance(double ref_speed_ms);
  double getSpeedLimit();
  double getCurvatureSpeed(int index);
  double getAvoidSpeed(int index);
  double getCutInSpeed();
  double getBezierSpeed();
  double getEndOfPathSpeed();
  double getTargetSpeed(int index);
  void setTargetSpeed(double target_speed);

  /* Object */
  void initializeStopNodes();
  void generateStopNodes();
  void transformObject(const obj_msgs::Obj src, obj_msgs::Obj &dest);
  void transformAtypicalObject(obj_msgs::Obj &obj);
  void locateEgo();
  void locateObj(obj_msgs::Obj &obj);
  void publishLocation();
  void publishRoadState();
  double distBtwObjs(obj_msgs::Obj obj_1, obj_msgs::Obj obj_2);
  bool collisionWithApproachingObject(double range);
  obj_msgs::Obj getEllipse(obj_msgs::Obj obj, geometry_msgs::Point center, double scale);
  void checkAvoidance();

  /* path generation */
  void generateDynamicPath();
  void generateCandidates(double min_offset, int safety_index);
  Candidate generatePathCandidate(double offset, int safety_index);
  void convertPathToCartesian();
  void generateCenterLine();
  void setObsCost();
  void setR(int idx, bool is_obs, std::vector<path_msgs::Lane> lanes);
  void setOffsetCost();
  void setContinuityCost();
  double getSafetyDistance();
  int getLateralDir(double x, double y, double sk);
  int distanceToIndex(tk::spline s, int idx, double dist);
  // int selectOptimalPath();
  void findOptimalPath();
  std::vector<double> getCollisionRisk();
  LControl laneStatus(path_msgs::Link link);
  std::tuple<double, double> getSP(obj_msgs::Obj obj);
  std::string getLinkID(obj_msgs::Obj obj);
  std::tuple<std::vector<path_msgs::Lane>, std::vector<path_msgs::Lane>> getAdjacentLanes(std::string link_id);
  void checkPath(std::vector<geometry_msgs::Point> points,
                                  std::vector<path_msgs::Lane> adj_lanes,
                                  int cand_idx);
  void publishReferencePath(std::vector<geometry_msgs::Point> path);
  Candidate getPathFollowingCandidate();


  void startThread();
  // State switchState();
  void switchState();
  void switchRelaying();
  void switchToGlobal(obj_msgs::Obj obj);

  /* miscellaneous */
  std::tuple<double, double> locateObjectFromEgo(obj_msgs::Obj obj);
  std::tuple<double, double> locateObjectBackFromEgo(obj_msgs::Obj obj);
  void printObjStatus();
  bool isOnPath();
  bool isDeadEnd();
  bool isBlocked();
  bool isReached();
  bool isVehicle(obj_msgs::Obj obj);
  bool isVehicleAround();
  bool isCollision(double x, double y, obj_msgs::Obj obj);
  bool isSafeLane(path_msgs::Lane lane);
  std::tuple<bool,bool,bool> isBeforeLTG(obj_msgs::Obj obj, std::tuple<double,double,double> thresholds);
  bool isAtIntersection(int index);
  bool isAtLaneChange(int index);
  std::string searchNextLink(int index);
  int searchTrajIndex(obj_msgs::Obj obj, path_msgs::Trajectory traj, double locate_range);
  int searchLinkIndex(geometry_msgs::Point point, path_msgs::Link link);
  int searchLinkIndexByLength(path_msgs::Link link, int from_idx, int to_idx, double refer_length);
  double calcLength(path_msgs::Link, int from_idx, int to_idx);
  void calcRelativeYaw();
  void getBezierInfo();
  double getDistance(double x1, double y1, double x2, double y2);
  double getAngleDiff(double first_angle, double second_angle);
};

#endif
