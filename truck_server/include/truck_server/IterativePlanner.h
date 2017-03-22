#ifndef ITERATIVE_PLANNER_H_
#define ITERATIVE_PLANNER_H_

#include <ros/ros.h>
#include <truck_server/TruckOctomapServer.h>
#include <unistd.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <quadrotor_trajectory/TrackParamStamped.h>
#include <iostream>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_trajectory/VehicleTrajectoryBase.h>
#include <truck_server/QuadrotorCommand.h>
// Already defined in truck_server/QuadrotorCommand.h
// #include <bspline_ros/bsplineGenerate.h>

using namespace octomap_server;
using namespace vehicle_trajectory_base;

class IterativePlanner
{
public:
  ros::NodeHandle m_nh;
  /* Subscriber */
  ros::Subscriber m_sub_lane_marker_flag;
  ros::Subscriber m_sub_target_traj_param;
  ros::Subscriber m_sub_target_odom;
  ros::Subscriber m_sub_uav_start_flag;
  ros::Subscriber m_sub_uav_odom;
  ros::Subscriber m_sub_uav_straight_lane_landing_start_flag;
  ros::Subscriber m_sub_uav_arrive_gps_point_flag;

  /* Publisher */
  ros::Publisher m_pub_uav_cmd;
  ros::Publisher m_pub_reconstructed_path_markers;
  ros::Publisher m_pub_estimated_target_odom;


  /* Visualization related */
  double m_octomap_res;
  int m_octomap_tree_depth;
  double m_octomap_boarder_val;
  double m_vehicle_traj_recv_time;
  double m_global_planning_period_default_time;
  double m_global_planning_period_time;
  double m_vehicles_visualize_prev_time;
  double m_vehicles_visualize_period_time;
  // 1 is circle track, 2 has bridge, 3 has crossing car, 4 has bridge and crossing car
  int m_route_id;
  bool m_collision_detection_flag;
  bool m_dji_mode;
  bool m_debug_mode;
  bool m_landing_mode;
  bool m_gazebo_mode;

  /* target information */
  nav_msgs::Odometry m_target_odom;
  std::string m_target_odom_sub_topic_name;
  VehicleTrajectoryBase m_target_traj_base;
  TruckOctomapServer* m_target_ptr;
  bool m_target_traj_param_print_flag;

  /* bspline generator */
  bsplineGenerate m_bspline_generator;
  int m_spline_degree;
  std::string m_spline_path_pub_topic_name;

  /* iterative searching */
  std::vector<TruckOctomapServer*> m_object_seg_ptr_vec;
  int m_n_segments;
  int m_n_total_segments;
  double m_landing_time;
  double m_target_height;
  double m_default_landing_vel;
  double m_segment_period_time;
  std::vector<Vector3d> m_control_point_vec;
  double m_uav_default_upbound_vel;
  double m_uav_landing_time_xy_upbound;
  double m_uav_default_tracking_time;

  /* uav */
  bool m_uav_direct_start_mode;
  nav_msgs::Odometry m_uav_odom;
  QuadrotorCommand m_uav;
  std::string m_uav_odom_sub_topic_name;
  std::string m_uav_cmd_pub_topic_name;
  double m_uav_landing_constant_vel;
  double m_uav_force_landing_vel;
  int m_uav_force_landing_method;
  double m_uav_force_landing_start_time;

  /* restricted region */
  bool m_restricted_region_mode;
  bool m_restricted_region_recv_flag;
  Vector3d m_restricted_region_center_pos;
  double m_restricted_region_radius;
  ros::Subscriber m_sub_restricted_region_center;
  void restrictedRegionCenterCallback(const nav_msgs::OdometryConstPtr& msg);
  void restrictedControlPoint(Vector3d& pt);

  void onInit();

  /* callback function */
  void laneMarkerCallback(const std_msgs::Empty msg);
  void targetTrajParamCallback(const quadrotor_trajectory::TrackParamStampedConstPtr& msg);
  void targetOdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /* octomap */
  bool isInsideBoarder(point3d query_point); // return true if the grid is free
  bool isInsideBoarder(Vector3d query_point);
  bool getGridCenter(point3d query_point, point3d& center_point, int depth);
  bool getGridCenter(Vector3d query_point, Vector3d& center_point, int depth);
  bool getGridCenter(TruckOctomapServer* obstacle_ptr, Vector3d query_point, Vector3d& center_point, int depth);
  inline void vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32);
  inline void vector3dConvertToPoint(Vector3d point3, geometry_msgs::Point& point32);

  /* uav */
  void uavStartFlagCallback(const std_msgs::Empty msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void uavStraightLaneLandingStartFlagCallback(const std_msgs::Empty msg);
  void uavArriveGpsPointFlagCallback(const std_msgs::Empty msg);

  // test
  void uavRandomCheatOdom();
  void controlPtsRandomSet();

  // iterative search
  void runIterativeSearching();
  void initIterativeSearching();
  void onIterativeSearching();

  /* visualization */
  void controlPolygonDisplay(int mode);
  void vehicleCurrentPosVisualization(int vehicle_type);
};

#endif
