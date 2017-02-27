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
#include <iostream>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_trajectory/VehicleTrajectoryBase.h>
#include <bspline_ros/bsplineGenerate.h>

using namespace octomap_server;
using namespace vehicle_trajectory_base;

struct aStarDataType
{
  point3d pos;
  double g_val;
  double h_val;
  double f_val;
  int ang_id;
  int id;
  int prev_id;
};
struct aStarComparator {
  bool operator()(aStarDataType i, aStarDataType j) {
    return i.f_val > j.f_val;
  }
};

class TruckServerNode
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_point_occupied_query_;
  ros::Subscriber sub_lane_marker_flag_;
  ros::Subscriber sub_point_depth_query_;
  ros::Subscriber sub_astar_query_;
  ros::Subscriber sub_cars_poses_;
  ros::Subscriber sub_truck_traj_param_;
  ros::Subscriber sub_car_inner_traj_param_;
  ros::Subscriber sub_car_outter_traj_param_;
  ros::Subscriber sub_truck_odom_;
  ros::Subscriber sub_car_inner_odom_;
  ros::Subscriber sub_car_outter_odom_;

  ros::Publisher pub_point_octocube_;
  ros::Publisher pub_reconstructed_path_markers_;
  ros::Publisher pub_control_points_;

  std::vector<aStarDataType> open_set_vec_, close_set_vec_, data_set_vec_;
  int data_set_num_;
  std::vector<point3d> astar_path_vec_;
  point3d m_init_point, m_land_point;
  float spline_res;

  bool m_has_car_inner;
  bool m_has_car_outter;

  int m_octomap_update_feq;
  int m_octomap_update_feq_cnt;
  int m_vehicle_octomap_visualize_mode;

  int m_octo_publish_all_cnt;

  double m_octomap_res;
  int m_octomap_tree_depth;
  double m_octomap_boarder_val;

  double m_ara_star_rate;
  int m_graph_connected_mode;
  std::vector<Vector3d> m_seach_graph_connected_map;

  nav_msgs::Odometry m_truck_odom;
  nav_msgs::Odometry m_car_inner_odom;
  nav_msgs::Odometry m_car_outter_odom;
  double m_vehicle_traj_recv_time;
  double m_global_planning_period_time;
  double m_vehicles_visualize_prev_time;
  double m_vehicles_visualize_period_time;
  std_msgs::Float64MultiArray m_car_inner_traj_msg;
  std_msgs::Float64MultiArray m_car_outter_traj_msg;
  bool m_car_inner_traj_msg_recv_flag;
  bool m_car_outter_traj_msg_recv_flag;

  int m_car_inner_type;
  int m_car_outter_type;
  /* 1 is circle track, 2 has bridge, 3 has crossing car, 4 has bridge and crossing car */
  int m_route_id;

  VehicleTrajectoryBase m_truck_traj_base;
  VehicleTrajectoryBase m_car_inner_traj_base;
  VehicleTrajectoryBase m_car_outter_traj_base;

  TruckOctomapServer* m_truck_ptr;

  /* bspline generator */
  bsplineGenerate m_bspline_generator;
  int m_spline_degree;
  std::string m_spline_path_pub_topic_name;

  /* iterative searching */
  std::vector<TruckOctomapServer*> m_object_seg_ptr_vec;
  int m_n_segments;
  double m_landing_time;
  double m_target_height;
  double m_default_landing_vel;
  double m_segment_period_time;
  std::vector<Vector3d> m_control_point_vec;
  double m_uav_default_upbound_vel;

  nav_msgs::Odometry m_uav_odom;

  /* Ground truth */
  double m_route_radius_gt;
  double m_truck_vel_gt;
  double m_car_outter_vel_gt;
  double m_car_inner_vel_gt;
  Vector3d nOrderVehicleTrajectoryFromGroundTruth(int order, double t0);
  void updateObstacleOctomapFromGroundTruth(TruckOctomapServer* obstacle_ptr, double t0);
  bool m_ground_truth_flag;

  void pointOccupiedQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void laneMarkerCallback(const std_msgs::Empty msg);
  void astarPathQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void truckTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  void carInnerTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  void carOutterTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  void truckOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void carInnerOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void carOutterOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void vehicleCurrentPosVisualization(int vehicle_type);
  void obstacleCurrentPosVisualization(int obstacle_type);

  void onInit();
  void aStarSearchGraphInit();
  // return true if the grid is free
  bool isInsideBoarder(point3d query_point);
  bool isInsideBoarder(Vector3d query_point);
  bool getGridCenter(point3d query_point, point3d& center_point, int depth);
  bool getGridCenter(Vector3d query_point, Vector3d& center_point, int depth);
  bool getGridCenter(TruckOctomapServer* obstacle_ptr, Vector3d query_point, Vector3d& center_point, int depth);
  bool aStarSearchInit();
  bool aStarSearch();
  std::vector<aStarDataType>::iterator getPosItearator(double f_val, char ch);
  bool nodeInCloseSet(aStarDataType& node);
  bool nodeInOpenSet(aStarDataType& node);
  void reconstructPath(int end_id);
  void reconstructedPathDisplay(int mode); // mode: 1, add display; -1, delete display
  inline void vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32);
  inline void vector3dConvertToPoint(Vector3d point3, geometry_msgs::Point& point32);

  // test
  void runAstarTest();
  void uavRandomCheatOdom();
  void controlPtsRandomSet();

  // iterative search
  void runIterativeSearching();
  void initIterativeSearching();
  void onIterativeSearching();
  void updateObstacleOctomap(TruckOctomapServer* obstacle_ptr, double t0);
  void controlPolygonDisplay(int mode);
};

void TruckServerNode::onInit()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("resolution", m_octomap_res, 0.1);
  private_nh.param("tree_depth", m_octomap_tree_depth, 16);
  private_nh.param("route_id", m_route_id, 1);
  private_nh.param("has_car_inner", m_has_car_inner, true);
  private_nh.param("has_car_outter", m_has_car_outter, true);
  private_nh.param("vehilce_inner_type", m_car_inner_type, 1);
  private_nh.param("spline_degree", m_spline_degree, 2);
  private_nh.param("vehilce_outter_type", m_car_outter_type, 2);
  private_nh.param("octomap_update_rate", m_octomap_update_feq, 1);
  private_nh.param("ARA_rate", m_ara_star_rate, 1.0);
  private_nh.param("graph_connected_mode", m_graph_connected_mode, 27);
  private_nh.param("vehicles_visualize_period_time", m_vehicles_visualize_period_time, 0.5);
  private_nh.param("global_planning_period_time", m_global_planning_period_time, 1.0);
  private_nh.param("target_height", m_target_height, 0.8);
  private_nh.param("uav_default_upbound_vel", m_uav_default_upbound_vel, 5.0);
  private_nh.param("spline_path_pub_topic_name", m_spline_path_pub_topic_name, (std::string)"spline_path");

  /*Ground truth */
  private_nh.param("ground_truth_flag", m_ground_truth_flag, true);
  private_nh.param("route_radius_ground_truth", m_route_radius_gt, 30.0);
  private_nh.param("truck_vel_ground_truth", m_truck_vel_gt, 4.17);
  private_nh.param("car_outter_ground_truth", m_car_outter_vel_gt, 4.17);
  private_nh.param("car_inner_ground_truth", m_car_inner_vel_gt, 4.17);

  m_bspline_generator.onInit(m_spline_degree, true, m_spline_path_pub_topic_name);

  /* 0 is only visualize vehicles current position, 1 is visualize their future position with connected octomap. */
  private_nh.param("vehicle_octomap_visualize_mode", m_vehicle_octomap_visualize_mode, 0);

  m_truck_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth);

  m_octomap_update_feq_cnt = 0;
  m_octo_publish_all_cnt = 0;
  m_octomap_boarder_val = m_octomap_res * pow(2, m_octomap_tree_depth-1);

  m_car_inner_traj_msg_recv_flag = false;
  m_car_outter_traj_msg_recv_flag = false;

  aStarSearchGraphInit();

  /* Subscriber */
  sub_point_occupied_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_occupied", 10, &TruckServerNode::pointOccupiedQueryCallback, this);
  sub_lane_marker_flag_ = nh_.subscribe<std_msgs::Empty>("/lane_marker_flag", 1, &TruckServerNode::laneMarkerCallback, this);
  sub_point_depth_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_depth", 1, &TruckServerNode::pointDepthQueryCallback, this);
  sub_truck_traj_param_ = nh_.subscribe<std_msgs::Float64MultiArray>("/truck_traj_param", 1, &TruckServerNode::truckTrajParamCallback, this);
  sub_car_inner_traj_param_ = nh_.subscribe<std_msgs::Float64MultiArray>("/car_inner_traj_param", 1, &TruckServerNode::carInnerTrajParamCallback, this);
  sub_car_outter_traj_param_ = nh_.subscribe<std_msgs::Float64MultiArray>("/car_outter_traj_param", 1, &TruckServerNode::carOutterTrajParamCallback, this);
  sub_truck_odom_ = nh_.subscribe<nav_msgs::Odometry>("/truck_odom", 1, &TruckServerNode::truckOdomCallback, this);
  sub_car_inner_odom_ = nh_.subscribe<nav_msgs::Odometry>("/car_inner_odom", 1, &TruckServerNode::carInnerOdomCallback, this);
  sub_car_outter_odom_ = nh_.subscribe<nav_msgs::Odometry>("/car_outter_odom", 1, &TruckServerNode::carOutterOdomCallback, this);

  /* Publisher */
  pub_point_octocube_ = nh_.advertise<visualization_msgs::Marker>("octo_cube_marker", 1);
  pub_reconstructed_path_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  pub_control_points_  = nh_.advertise<geometry_msgs::PolygonStamped>("path_gird_points", 1);

  std::cout << "onInit finished.\n";
  ROS_INFO("onInit");

  m_vehicles_visualize_prev_time = ros::Time().now().toSec();
  m_vehicle_traj_recv_time = m_vehicles_visualize_prev_time;

  // todo: get from launch
  spline_res = 0.2f;
}

void TruckServerNode::initIterativeSearching()
{
  /* Clear previous vector data, marker, octree data */
  if (!m_control_point_vec.empty()){
    controlPolygonDisplay(0);
    m_control_point_vec.clear();
  }
  for (int i = 0; i < m_object_seg_ptr_vec.size(); ++i)
    m_object_seg_ptr_vec[i]->m_octree->clear();
  m_object_seg_ptr_vec.clear();

  /* Initialize segment estimated landing time, which could be dynamically adjusted when meeting obstacles */
  double landing_time_z = (m_uav_odom.pose.pose.position.z - m_target_height) / 1.0;
  // if current z speed is nearly 0, then we assume there is an acceleration to 1 m/s taking 1 second
  if (m_uav_odom.twist.twist.linear.z < 0.1)
    landing_time_z += 1.0;
  double truck_vel = sqrt(pow(m_truck_odom.twist.twist.linear.x, 2) +pow(m_truck_odom.twist.twist.linear.y, 2));
  double landing_time_xy = sqrt(pow(m_uav_odom.pose.pose.position.x-m_truck_odom.pose.pose.position.x, 2) + pow(m_uav_odom.pose.pose.position.y-m_truck_odom.pose.pose.position.y, 2)) / (m_uav_default_upbound_vel - truck_vel);
  // if result is 5.1 s, we will use 6.0s as the landing time
  if (landing_time_z > landing_time_xy)
    m_landing_time = (int)(landing_time_z + 0.99);
  else
    m_landing_time = (int)(landing_time_xy + 0.99);

  /* Initialize segment default period time, which is 1.0s */
  m_segment_period_time = 1.0;

  Vector3d control_pt_0(m_uav_odom.pose.pose.position.x, m_uav_odom.pose.pose.position.y, m_uav_odom.pose.pose.position.z);
  m_control_point_vec.push_back(control_pt_0);
  // Set second control point: P1 = P0 + v * Tp / 2
  while(1){
    TruckOctomapServer* obstacle_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
    if (m_ground_truth_flag)
      updateObstacleOctomapFromGroundTruth(obstacle_ptr, 0);
    else
      updateObstacleOctomap(obstacle_ptr, 0);
    Vector3d control_pt_1 = control_pt_0 +
      Vector3d(m_uav_odom.twist.twist.linear.x*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.y*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.z*m_segment_period_time/2.0);
    Vector3d temp_3d;
    // todo: collision detection
    m_object_seg_ptr_vec.push_back(obstacle_ptr);
    m_control_point_vec.push_back(control_pt_1);
    /* if we have 2 segments on z axis, we have 3 segments in total (because of first and last point and its neighbor control points generating based on velocity constraits) */
    m_landing_time += m_segment_period_time;
    m_n_segments = int(m_landing_time / m_segment_period_time);
    break;

    // if (getGridCenter(obstacle_ptr, control_pt_1, temp_3d, -1)){
    //   m_object_seg_ptr_vec.push_back(obstacle_ptr);
    //   m_control_point_vec.push_back(control_pt_1);
    //   /* if we have 2 segments on z axis, we have 3 segments in total (because of first and last point and its neighbor control points generating based on velocity constraits) */
    //   m_landing_time += m_segment_period_time;
    //   m_n_segments = int(m_landing_time / m_segment_period_time);
    //   break;
    // }
    // else{
    //   ROS_WARN("Control points 1 could not use!! Decrease initial gap to its 1/2.");
    //   m_segment_period_time /= 2.0;
    //   obstacle_ptr->m_octree->clear();
    // }
    // todo: check land point's forhead control point to decide segment_period_time
  }
}

void TruckServerNode::onIterativeSearching()
{

  /* Simple generate control points, ignoring obstacles avoidance */
  Vector3d target_cur_pt(m_truck_odom.pose.pose.position.x, m_truck_odom.pose.pose.position.y, m_target_height);
  for (int i = 1; i <= m_n_segments-1; ++i){
    Vector3d cur_control_pt;
    if (m_ground_truth_flag)
      cur_control_pt = (m_control_point_vec[0] - target_cur_pt) / (m_n_segments-1) * (m_n_segments-1-i) + nOrderVehicleTrajectoryFromGroundTruth(0, i*m_segment_period_time) + Vector3d(0.0, 0.0, m_target_height);
    else
      cur_control_pt = (m_control_point_vec[0] - target_cur_pt) / (m_n_segments-1) * (m_n_segments-1-i) + m_truck_traj_base.nOrderVehicleTrajectory(0, i*m_segment_period_time) + Vector3d(0.0, 0.0, m_target_height);
    m_control_point_vec.push_back(cur_control_pt);
  }

  /* Set second control point: Pn-1 = Pn - v * Tp / 2 */
  /* Simple add last 2 control points without collision checking */
  // todo: check last 2 control points
  while(1){
    TruckOctomapServer* obstacle_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
    Vector3d control_pt_n;
    Vector3d control_pt_n_1;
    if (m_ground_truth_flag){
      updateObstacleOctomapFromGroundTruth(obstacle_ptr, m_landing_time-m_segment_period_time);
      control_pt_n = nOrderVehicleTrajectoryFromGroundTruth(0, m_landing_time) + Vector3d(0.0, 0.0, m_target_height);
    // todo: adding landing speed. Currently z-axis value is 0 in last triangle.
      control_pt_n_1 = control_pt_n -
        nOrderVehicleTrajectoryFromGroundTruth(1, m_landing_time) * m_segment_period_time/2.0;
    }
    else{
      updateObstacleOctomap(obstacle_ptr, m_landing_time-m_segment_period_time);
      control_pt_n = m_truck_traj_base.nOrderVehicleTrajectory(0, m_landing_time) + Vector3d(0.0, 0.0, m_target_height);
    // todo: adding landing speed. Currently z-axis value is 0 in last triangle.
    control_pt_n_1 = control_pt_n -
      m_truck_traj_base.nOrderVehicleTrajectory(1, m_landing_time) * m_segment_period_time/2.0;
    }

    Vector3d temp_3d;
    /* Check control points whether is safe */
    // todo: stretegy when it happens
    if (getGridCenter(obstacle_ptr, control_pt_n, temp_3d, -1)){
      ROS_ERROR("Landing point is in collision!!!");
    }
    /* Check landing point's neighbor control point being safe */
    // todo: strategy when this point is in collision
    if (!getGridCenter(obstacle_ptr, control_pt_n_1, temp_3d, -1)){
      ROS_WARN("Control points n-1 could not use!! TODO work.");
    }
    m_control_point_vec.push_back(control_pt_n_1);
    m_control_point_vec.push_back(control_pt_n);
    break;
  }
}

void TruckServerNode::runIterativeSearching()
{
  // test
  /* uav odom cheat mode */
  uavRandomCheatOdom();

  initIterativeSearching();
  onIterativeSearching();

  /* test: specific control points test */
  //controlPtsRandomSet();

  /* publish control points */
  controlPolygonDisplay(1);

  /* Print the value of control points */
  for (int i = 0; i < m_control_point_vec.size(); ++i){
    std::cout << "[Control pt] " << i << ": " << m_control_point_vec[i].x() << ", "
              << m_control_point_vec[i].y() << ", " << m_control_point_vec[i].z() << "\n";
  }
}

void TruckServerNode::uavRandomCheatOdom()
{
  m_uav_odom = m_truck_odom;
  // position: truck behind 4.0m, height is 5m
  m_uav_odom.pose.pose.position.z = 4;
  m_uav_odom.pose.pose.position.x += 1.0 + (-3.0) * cos(m_uav_odom.pose.pose.orientation.w);
  m_uav_odom.pose.pose.position.y += 1.0 + (-3.0) * sin(m_uav_odom.pose.pose.orientation.w);
  // velocity: 0, 0, 0
  m_uav_odom.twist.twist.linear.x = 5.0;
  m_uav_odom.twist.twist.linear.y = 5.0;
  m_uav_odom.twist.twist.linear.z = 0.0;
}

void TruckServerNode::controlPtsRandomSet()
{
  /* test: specific control points test */
  if (!m_control_point_vec.empty()){
    controlPolygonDisplay(0);
    m_control_point_vec.clear();
  }
  m_segment_period_time = 1.0;
  Vector3d pt;
  pt = Vector3d(0, 0, 8); m_control_point_vec.push_back(pt);
  pt = Vector3d(2, 1, 7); m_control_point_vec.push_back(pt);
  pt = Vector3d(3, 2, 6); m_control_point_vec.push_back(pt);
  pt = Vector3d(2, 4, 4); m_control_point_vec.push_back(pt);
  pt = Vector3d(1, 5, 3); m_control_point_vec.push_back(pt);
  pt = Vector3d(0, 7, 1); m_control_point_vec.push_back(pt);
  pt = Vector3d(-1, 8, 1); m_control_point_vec.push_back(pt);
}

void TruckServerNode::updateObstacleOctomap(TruckOctomapServer* obstacle_ptr, double t0)
{
  // Add inner car
  if (m_has_car_inner){
    Vector3d car_inner_pos = m_car_inner_traj_base.nOrderVehicleTrajectory(0, t0);
    Vector3d car_inner_vel = m_car_inner_traj_base.nOrderVehicleTrajectory(1, t0);
    obstacle_ptr->WriteVehicleOctree(m_car_inner_type, Pose6D(car_inner_pos.x(), car_inner_pos.y(), 0.0f, 0.0, 0.0, atan2(car_inner_vel.y(), car_inner_vel.x())));
    car_inner_pos = m_car_inner_traj_base.nOrderVehicleTrajectory(0, t0 + m_segment_period_time);
    car_inner_vel = m_car_inner_traj_base.nOrderVehicleTrajectory(1, t0 + m_segment_period_time);
    obstacle_ptr->WriteVehicleOctree(m_car_inner_type, Pose6D(car_inner_pos.x(), car_inner_pos.y(), 0.0f, 0.0, 0.0, atan2(car_inner_vel.y(), car_inner_vel.x())));
  }
    // Add outter car
  if (m_has_car_outter){
    Vector3d car_outter_pos = m_car_outter_traj_base.nOrderVehicleTrajectory(0, t0);
    Vector3d car_outter_vel = m_car_outter_traj_base.nOrderVehicleTrajectory(1, t0);
    obstacle_ptr->WriteVehicleOctree(m_car_outter_type, Pose6D(car_outter_pos.x(), car_outter_pos.y(), 0.0f, 0.0, 0.0, atan2(car_outter_vel.y(), car_outter_vel.x())));
    car_outter_pos = m_car_outter_traj_base.nOrderVehicleTrajectory(0, t0 + m_segment_period_time);
    car_outter_vel = m_car_outter_traj_base.nOrderVehicleTrajectory(1, t0 + m_segment_period_time);
    obstacle_ptr->WriteVehicleOctree(m_car_outter_type, Pose6D(car_outter_pos.x(), car_outter_pos.y(), 0.0f, 0.0, 0.0, atan2(car_outter_vel.y(), car_outter_vel.x())));
  }


  /* 1 is circle track, 2 has bridge, 3 has crossing car, 4 has bridge and crossing car */
  switch (m_route_id){
  case 2:
    obstacle_ptr->WriteObstacleOctree(0, Pose6D(0.0f, -30.0f, 0.0f, 0.0, 0.0, 0.0));
    break;
  case 3:
    break;
  case 4:
    break;
  default:
    break;
  }
}

Vector3d TruckServerNode::nOrderVehicleTrajectoryFromGroundTruth(int order, double t0)
{
  double ang = atan2(m_truck_odom.pose.pose.position.x, -m_truck_odom.pose.pose.position.y);
  double r = m_route_radius_gt;
  double ang_vel = m_truck_vel_gt / r;
  if (order == 0){
    Vector3d truck_pos = Vector3d(r*sin(ang + ang_vel*t0), -r*cos(ang + ang_vel*t0), 0);
    return truck_pos;
  }
  else{
    Vector3d truck_vel = Vector3d(m_truck_vel_gt*cos(ang + ang_vel*t0), m_truck_vel_gt*sin(ang + ang_vel*t0), 0);
    return truck_vel;
  }
}

void TruckServerNode::updateObstacleOctomapFromGroundTruth(TruckOctomapServer* obstacle_ptr, double t0)
{
  // Add inner car
  if (m_has_car_inner){
    double ang = atan2(m_car_inner_odom.pose.pose.position.x, -m_car_inner_odom.pose.pose.position.y);
    double r = m_route_radius_gt - 5;
    double ang_vel = m_car_inner_vel_gt / r;
    Vector3d car_inner_pos = Vector3d(r*sin(ang + ang_vel*t0), -r*cos(ang + ang_vel*t0), 0);
    Vector3d car_inner_vel = Vector3d(m_car_inner_vel_gt*cos(ang + ang_vel*t0), m_car_inner_vel_gt*sin(ang + ang_vel*t0), 0);
    obstacle_ptr->WriteVehicleOctree(m_car_inner_type, Pose6D(car_inner_pos.x(), car_inner_pos.y(), 0.0f, 0.0, 0.0, atan2(car_inner_vel.y(), car_inner_vel.x())));
    car_inner_pos = m_car_inner_traj_base.nOrderVehicleTrajectory(0, t0 + m_segment_period_time);
    car_inner_vel = m_car_inner_traj_base.nOrderVehicleTrajectory(1, t0 + m_segment_period_time);
    obstacle_ptr->WriteVehicleOctree(m_car_inner_type, Pose6D(car_inner_pos.x(), car_inner_pos.y(), 0.0f, 0.0, 0.0, atan2(car_inner_vel.y(), car_inner_vel.x())));
  }
    // Add outter car
  if (m_has_car_outter){
    double ang = atan2(m_car_outter_odom.pose.pose.position.x, -m_car_outter_odom.pose.pose.position.y);
    double r = m_route_radius_gt + 5;
    double ang_vel = m_car_outter_vel_gt / r;
    Vector3d car_outter_pos = Vector3d(r*sin(ang + ang_vel*t0), -r*cos(ang + ang_vel*t0), 0);
    Vector3d car_outter_vel = Vector3d(m_car_outter_vel_gt*cos(ang + ang_vel*t0), m_car_outter_vel_gt*sin(ang + ang_vel*t0), 0);
    obstacle_ptr->WriteVehicleOctree(m_car_outter_type, Pose6D(car_outter_pos.x(), car_outter_pos.y(), 0.0f, 0.0, 0.0, atan2(car_outter_vel.y(), car_outter_vel.x())));
    car_outter_pos = m_car_outter_traj_base.nOrderVehicleTrajectory(0, t0 + m_segment_period_time);
    car_outter_vel = m_car_outter_traj_base.nOrderVehicleTrajectory(1, t0 + m_segment_period_time);
    obstacle_ptr->WriteVehicleOctree(m_car_outter_type, Pose6D(car_outter_pos.x(), car_outter_pos.y(), 0.0f, 0.0, 0.0, atan2(car_outter_vel.y(), car_outter_vel.x())));
  }
}

void TruckServerNode::pointOccupiedQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("Query");
  std::cout << "query comes.\n";
  point3d query(msg->x, msg->y, msg->z);
  OcTreeNode* result = m_truck_ptr->m_octree->search (query);
  if(result == NULL)
    {
      std::cout << "Free/Unknown point" << query << std::endl;
    }
  else
    {
      std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
      std::cout << "Logodds at " << query << ":\t " << result->getLogOdds() << std::endl << std::endl;
    }
}

void TruckServerNode::laneMarkerCallback(const std_msgs::Empty msg)
{
  m_truck_ptr->laneMarkerVisualization();
}


void TruckServerNode::vehicleCurrentPosVisualization(int vehicle_type)
{
  // x,y,z,r,p,y
  // todo: currently directly assign ang value to orientation.w
  if (vehicle_type == 0){
    m_truck_ptr->WriteVehicleOctree(0, Pose6D(m_truck_odom.pose.pose.position.x+0.8, m_truck_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_truck_odom.pose.pose.orientation.w));
  }
  // truck withour roof
  else if (vehicle_type == -1){
    m_truck_ptr->WriteVehicleOctree(-1, Pose6D(m_truck_odom.pose.pose.position.x+0.8, m_truck_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_truck_odom.pose.pose.orientation.w));
  }
  // small car
  else if (vehicle_type == 1){
    m_truck_ptr->WriteVehicleOctree(1, Pose6D(m_car_inner_odom.pose.pose.position.x+0.8, m_car_inner_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_car_inner_odom.pose.pose.orientation.w));
  }
  // big car
  else if (vehicle_type == 2){
    m_truck_ptr->WriteVehicleOctree(2, Pose6D(m_car_outter_odom.pose.pose.position.x+0.8, m_car_outter_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_car_outter_odom.pose.pose.orientation.w));
  }
}

void TruckServerNode::obstacleCurrentPosVisualization(int obstacle_type)
{
  /* 0 is truck */
  switch (obstacle_type){
  case 0:
    m_truck_ptr->WriteObstacleOctree(obstacle_type, Pose6D(0.0f, -30.0f, 0.0f, 0.0, 0.0, 0.0));
    break;
  case 2:
    break;
  default:
    break;
  }
}

void TruckServerNode::truckTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  double cur_time = ros::Time().now().toSec();
  if (cur_time - m_vehicle_traj_recv_time > m_global_planning_period_time){
    m_vehicle_traj_recv_time = cur_time;

    int traj_order = msg->layout.dim[0].size;
    std::vector<double> data;
    for (int i = 0; i < 2*traj_order+1; ++i)
      data.push_back(msg->data[i]);
    m_truck_traj_base.onInit(traj_order, data);

    // Load car inner's trajectory paramaters
    if (m_has_car_inner && m_car_inner_traj_msg_recv_flag){
      int traj_order1 = m_car_inner_traj_msg.layout.dim[0].size;
      std::vector<double> data1;
      for (int i = 0; i < 2*traj_order1+1; ++i)
        data1.push_back(m_car_inner_traj_msg.data[i]);
      m_car_inner_traj_base.onInit(traj_order1, data1);
    }

    // Load car outter's trajectory paramaters
    if (m_has_car_outter && m_car_outter_traj_msg_recv_flag){
      int traj_order1 = m_car_outter_traj_msg.layout.dim[0].size;
      std::vector<double> data1;
      for (int i = 0; i < 2*traj_order1+1; ++i)
        data1.push_back(m_car_outter_traj_msg.data[i]);
      m_car_outter_traj_base.onInit(traj_order1, data1);
    }
    /* run Iterative Searching */
    runIterativeSearching();
  }
}


void TruckServerNode::carInnerTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  m_car_inner_traj_msg = *msg;
  m_car_inner_traj_msg_recv_flag = true;

  // if (m_vehicle_octomap_visualize_mode == 1){
  //   for (int i = 0; i <=5; ++i){
  //     Vector3d vehicle_pos = m_car_inner_traj_base.nOrderVehicleTrajectory(0, i*1.0);
  //     Vector3d vehicle_vel = m_car_inner_traj_base.nOrderVehicleTrajectory(1, i*1.0);
  //     m_truck_ptr->WriteVehicleOctree(m_car_inner_type, Pose6D(vehicle_pos.x(), vehicle_pos.y(), 0.0f, 0.0, 0.0, atan2(vehicle_vel.y(), vehicle_vel.x())));
  //   }
}


void TruckServerNode::carOutterTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  m_car_outter_traj_msg = *msg;
  m_car_outter_traj_msg_recv_flag = true;
}

void TruckServerNode::pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("QueryDepth");
  point3d query(msg->x, msg->y, msg->z);
  std::cout << msg->x <<' ' << msg->y << ' ' << msg->z <<'\n';
  // Depth from 1 to 16
  int cur_depth;
  OcTreeNode* result = m_truck_ptr->m_octree->searchReturnDepth (query, 0, cur_depth);
  double cube_size = m_truck_ptr->m_octree->resolution * pow(2, m_truck_ptr->m_octree->tree_depth-cur_depth);
  std::cout << "Point is " << cur_depth << "\n";
  if (result == NULL)
    std::cout << "Unknown/free region.\n";
  else
    std::cout << "Prob is  " << result->getOccupancy() << "\n";
  point3d center;
  getGridCenter(query, center, cur_depth);

  visualization_msgs::Marker query_point_marker, octo_cube_marker;
  octo_cube_marker.ns = query_point_marker.ns = "octocubes";
  octo_cube_marker.header.frame_id = query_point_marker.header.frame_id = std::string("/world");
  octo_cube_marker.header.stamp = query_point_marker.header.stamp = ros::Time().now();
  octo_cube_marker.action = query_point_marker.action = visualization_msgs::Marker::ADD;
  octo_cube_marker.id = 0;
  query_point_marker.id = 1;
  octo_cube_marker.type = visualization_msgs::Marker::CUBE;
  query_point_marker.type = visualization_msgs::Marker::CUBE;


  query_point_marker.pose.position.x = msg->x;
  query_point_marker.pose.position.y = msg->y;
  query_point_marker.pose.position.z = msg->z;
  query_point_marker.pose.orientation.x = 0.0;
  query_point_marker.pose.orientation.y = 0.0;
  query_point_marker.pose.orientation.z = 0.0;
  query_point_marker.pose.orientation.w = 1.0;
  query_point_marker.scale.x = 0.1;
  query_point_marker.scale.y = 0.1;
  query_point_marker.scale.z = 0.1;
  query_point_marker.color.a = 1.0;
  query_point_marker.color.r = 0.8f;
  query_point_marker.color.g = 0.0f;
  query_point_marker.color.b = 0.0f;

  octo_cube_marker.pose.position.x = center.x();
  octo_cube_marker.pose.position.y = center.y();
  octo_cube_marker.pose.position.z = center.z();
  octo_cube_marker.pose.orientation.x = 0.0;
  octo_cube_marker.pose.orientation.y = 0.0;
  octo_cube_marker.pose.orientation.z = 0.0;
  octo_cube_marker.pose.orientation.w = 1.0;
  octo_cube_marker.scale.x = cube_size;
  octo_cube_marker.scale.y = cube_size;
  octo_cube_marker.scale.z = cube_size;
  octo_cube_marker.color.a = 0.5;
  octo_cube_marker.color.r = 0.0f;
  octo_cube_marker.color.g = 0.0f;
  octo_cube_marker.color.b = 1.0f;

  pub_point_octocube_.publish(query_point_marker);
  //sleep(1.5);
  pub_point_octocube_.publish(octo_cube_marker);

}

void TruckServerNode::controlPolygonDisplay(int mode){
  int control_points_num = m_control_point_vec.size();
  std::cout << "[Display] Control points number: " << control_points_num << "\n";
  int id_cnt = 0;
  visualization_msgs::MarkerArray path_markers;
  visualization_msgs::Marker control_point_marker, line_list_marker;
  // line_array for display truck ground truth future data, usually noted.
  visualization_msgs::Marker line_array_truck_gt_marker;
  control_point_marker.ns = line_list_marker.ns = "control_polygon";
  control_point_marker.header.frame_id = line_list_marker.header.frame_id = std::string("/world");
  control_point_marker.header.stamp = line_list_marker.header.stamp = ros::Time().now();
  if (mode == 1)
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::ADD;
  else
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::DELETE;

  line_array_truck_gt_marker.header = line_list_marker.header;
  line_array_truck_gt_marker.action = line_list_marker.action;
  line_array_truck_gt_marker.ns = line_list_marker.ns;

  control_point_marker.type = visualization_msgs::Marker::SPHERE;
  line_list_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_array_truck_gt_marker.type = visualization_msgs::Marker::LINE_STRIP;

  geometry_msgs::PolygonStamped control_polygon_points;
  control_polygon_points.header = control_point_marker.header;

  for (int i = 0; i < control_points_num; ++i){
    geometry_msgs::Point32 control_point, time_point;
    time_point.x = m_segment_period_time * i;
    control_polygon_points.polygon.points.push_back(time_point);
    vector3dConvertToPoint32(m_control_point_vec[i], control_point);
    control_polygon_points.polygon.points.push_back(control_point);
  }
  if (mode == 1){
    //pub_control_points_.publish(control_polygon_points);
    m_bspline_generator.bsplineParamInput(&control_polygon_points);
    /* Print knot velocity */
    m_bspline_generator.getDerive();
    std::cout << "Print Knot info. \n" ;
    for (int j = m_bspline_generator.m_deg; j < m_bspline_generator.m_n_knots - m_bspline_generator.m_deg; ++j){
      double knot_time = m_bspline_generator.m_knotpts[j];
      std::vector<double> knot_vel_vec = m_bspline_generator.evaluateDerive(knot_time);
      double knot_vel = sqrt(pow(knot_vel_vec[0],2) + pow(knot_vel_vec[1],2) + pow(knot_vel_vec[2],2));
      if (knot_vel > 10.0)
        ROS_ERROR("!!!!!!!!!!!!!!!!!!!! Knot vel is too large!!!!");
      std::cout << "[Knot] " << j << ": " << knot_time << ", vel: " << knot_vel << "|| "
                << knot_vel_vec[0] << ", "<< knot_vel_vec[1] << ", "<< knot_vel_vec[2] << "\n";
    }
  }

  line_list_marker.id = id_cnt;
  ++id_cnt;
  line_list_marker.scale.x = 0.1;
  line_list_marker.color.r = 0.0;
  line_list_marker.color.g = 1.0;
  line_list_marker.color.b = 0.0;
  line_list_marker.color.a = 1.0;
  geometry_msgs::Point pt;
  vector3dConvertToPoint(m_control_point_vec[0], pt);
  line_list_marker.points.push_back(pt);
  vector3dConvertToPoint(m_control_point_vec[1], pt);
  line_list_marker.points.push_back(pt);
  for (int i = 2; i < control_points_num; ++i){
    vector3dConvertToPoint(m_control_point_vec[i-2], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i-1], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i], pt);
    line_list_marker.points.push_back(pt);
  }
  path_markers.markers.push_back(line_list_marker);

  for (int i = 0; i < control_points_num; ++i){
    control_point_marker.id = id_cnt;
    ++id_cnt;
    control_point_marker.pose.position.x = m_control_point_vec[i].x();
    control_point_marker.pose.position.y = m_control_point_vec[i].y();
    control_point_marker.pose.position.z = m_control_point_vec[i].z();
    control_point_marker.pose.orientation.x = 0.0;
    control_point_marker.pose.orientation.y = 0.0;
    control_point_marker.pose.orientation.z = 0.0;
    control_point_marker.pose.orientation.w = 1.0;
    if (i == 0 || i == control_points_num-1){
      control_point_marker.scale.x = 1.0;
      control_point_marker.scale.y = 1.0;
      control_point_marker.scale.z = 1.0;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
      continue;
    }
    else{
      control_point_marker.scale.x = 0.5;
      control_point_marker.scale.y = 0.5;
      control_point_marker.scale.z = 0.5;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
    }
  }

  /* Add elements in line strip */
  // line_array_truck_gt_marker.id = id_cnt;
  // ++id_cnt;
  // line_array_truck_gt_marker.scale.x = 1.0;
  // line_array_truck_gt_marker.color.r = 1.0;
  // line_array_truck_gt_marker.color.g = 0.0;
  // line_array_truck_gt_marker.color.b = 0.0;
  // line_array_truck_gt_marker.color.a = 1.0;
  // double sample_gap = 0.1;
  // for (int i = 0; i < (int)(m_landing_time/sample_gap); ++i){
  //   geometry_msgs::Point pt;
  //   vector3dConvertToPoint(nOrderVehicleTrajectoryGroundTruth(0, sample_gap*i), pt);
  //   line_array_truck_gt_marker.points.push_back(pt);
  // }
  // path_markers.markers.push_back(line_array_truck_gt_marker);

  pub_reconstructed_path_markers_.publish(path_markers);
}

bool TruckServerNode::isInsideBoarder(Vector3d query_point)
{
  if (query_point.x() >= m_octomap_boarder_val || query_point.x() <= -m_octomap_boarder_val
      || query_point.y() >= m_octomap_boarder_val || query_point.y() <= -m_octomap_boarder_val
      ||query_point.z() >= m_octomap_boarder_val || query_point.z() <= -m_octomap_boarder_val)
    return false;
  else
    return true;
}

bool TruckServerNode::isInsideBoarder(point3d query_point)
{
  if (query_point.x() >= m_octomap_boarder_val || query_point.x() <= -m_octomap_boarder_val
      || query_point.y() >= m_octomap_boarder_val || query_point.y() <= -m_octomap_boarder_val
      ||query_point.z() >= m_octomap_boarder_val || query_point.z() <= -m_octomap_boarder_val)
    return false;
  else
    return true;
}

bool TruckServerNode::getGridCenter(TruckOctomapServer* obstacle_ptr, Vector3d query_point, Vector3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      point3d query_point_p(query_point[0], query_point[1], query_point[1]);
      OcTreeNode* result = obstacle_ptr->m_octree->searchReturnDepth(query_point_p, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }
  key_type key_x = obstacle_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = obstacle_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = obstacle_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = obstacle_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = obstacle_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = obstacle_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}


bool TruckServerNode::getGridCenter(Vector3d query_point, Vector3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      point3d query_point_p(query_point[0], query_point[1], query_point[1]);
      std::cout << "query point: " << query_point_p.x() << ", " << query_point_p.y() << ", " << query_point_p.z() << "\n";
      OcTreeNode* result = m_truck_ptr->m_octree->searchReturnDepth(query_point_p, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }
  key_type key_x = m_truck_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = m_truck_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = m_truck_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = m_truck_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = m_truck_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = m_truck_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}

bool TruckServerNode::getGridCenter(point3d query_point, point3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      OcTreeNode* result = m_truck_ptr->m_octree->searchReturnDepth(query_point, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }

  key_type key_x = m_truck_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = m_truck_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = m_truck_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = m_truck_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = m_truck_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = m_truck_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}

// bool TruckServerNode::aStarSearchInit()
// {
//   data_set_num_ = 0;
//   data_set_vec_.clear();
//   open_set_vec_.clear();
//   close_set_vec_.clear();
//   astar_path_vec_.clear();

//   point3d start_point;
//   if (!getGridCenter(m_init_point, start_point, -1)){
//     ROS_ERROR("Init astar search point is not available.");
//     return false;
//   }
//   int start_node_depth;
//   m_truck_ptr->m_octree->searchReturnDepth(start_point, 0, start_node_depth);
//   double start_grid_size = m_truck_ptr->m_octree->resolution * pow(2, m_truck_ptr->m_octree->tree_depth-start_node_depth);
//   double neighbor_grid_gap = start_grid_size/2.0 + m_truck_ptr->m_octree->resolution/2.0;
//   for (int i = 0; i < m_graph_connected_mode; ++i){
//     point3d neighbor_center_point;
//     Vector3d index = m_seach_graph_connected_map[i];
//     if (getGridCenter((start_point + point3d(neighbor_grid_gap*index[0],
//                                              neighbor_grid_gap*index[1],
//                                              neighbor_grid_gap*index[2])),
//                       neighbor_center_point, -1)){
//       if (neighbor_center_point.z() < 0)
//         continue;
//       for (int k = 0; k < 6;++k){
//         aStarDataType new_astar_node;
//         new_astar_node.prev_id = -1;
//         new_astar_node.id = data_set_num_;
//         /* k is 0xabc, means x axis is a (0:up, 1:down), y axis is b, z axis is c. */
//         new_astar_node.ang_id = k;
//         new_astar_node.pos = neighbor_center_point;
//         new_astar_node.g_val = m_init_point.distance(neighbor_center_point);
//         double ang_cost = 0;
//         /* if uav goes up in z axis */
//         if (k && 0x1 == 0)
//           ang_cost = new_astar_node.g_val;
//         new_astar_node.g_val += ang_cost;
//         // Euclidean distance
//         new_astar_node.h_val = m_ara_star_rate * neighbor_center_point.distance(m_land_point);
//         // Manhattan distance
//         // new_astar_node.h_val = m_ara_star_rate *
//         //   (fabs(neighbor_center_point.x()-m_land_point.x())
//         //    + fabs(neighbor_center_point.y()-m_land_point.y())
//         //    + fabs(neighbor_center_point.z()-m_land_point.z()));
//         new_astar_node.f_val = new_astar_node.g_val + new_astar_node.h_val;
//         open_set_vec_.insert(getPosItearator(new_astar_node.f_val, 'o'), new_astar_node);
//         data_set_vec_.push_back(new_astar_node);
//         ++data_set_num_;
//         //test
//         //astar_path_vec_.push_back(neighbor_center_point);
//       }
//     }
//   }
//   return true;
// }


// bool TruckServerNode::aStarSearch()
// {
//   if (!aStarSearchInit()){
//     ROS_INFO("A Star Search failed!!");
//     return false;
//   }
//   point3d end_point;
//   if (!getGridCenter(m_land_point, end_point, -1)){
//     ROS_ERROR("Land point is not available.");
//     ROS_INFO("A Star Search failed!!");
//     return false;
//   }

//   while (!open_set_vec_.empty())
//     {
//       //std::cout << "open set size: " << open_set_vec_.size() << '\n';
//       // Pop first element in open set, and push into close set
//       aStarDataType start_astar_node = open_set_vec_[0];
//       point3d start_point = start_astar_node.pos;
//       // Judge whether reach the goal point
//       // TODO: better end conditions is needed
//       if (start_point == end_point)
//         {
//           ROS_INFO("Reach the goal point.");
//           reconstructPath(start_astar_node.id);
//           return true;
//         }
//       close_set_vec_.insert(getPosItearator(start_astar_node.f_val, 'c'), start_astar_node);
//       open_set_vec_.erase(open_set_vec_.begin());

//       int start_octree_node_depth;
//       m_truck_ptr->m_octree->searchReturnDepth(start_point, 0, start_octree_node_depth);
//       double start_grid_size = m_truck_ptr->m_octree->resolution * pow(2, m_truck_ptr->m_octree->tree_depth-start_octree_node_depth);
//       double neighbor_grid_gap = start_grid_size/2.0 + m_truck_ptr->m_octree->resolution/2.0;
//       // Starts from 1, do not need first index, because this node is already visited
//       for (int i = 1; i < m_graph_connected_mode; ++i){
//         point3d neighbor_center_point;
//         Vector3d index = m_seach_graph_connected_map[i];
//         if (getGridCenter((start_point + point3d(neighbor_grid_gap*index[0],
//                                                  neighbor_grid_gap*index[1],
//                                                  neighbor_grid_gap*index[2])),
//                           neighbor_center_point, -1)){
//           if (neighbor_center_point.z() < 0)
//             continue;
//           /* If this neighbor is end_point, then stop */
//           else if (neighbor_center_point == end_point){
//             ROS_INFO("Reach the goal point.");
//             int end_grid_depth;
//             m_truck_ptr->m_octree->searchReturnDepth(end_point, 0, end_grid_depth);
//             double end_grid_size = m_truck_ptr->m_octree->resolution * pow(2, m_truck_ptr->m_octree->tree_depth-end_grid_depth);
//             /* if end grid is too large, skip it */
//             if (end_grid_size > 2.0){
//               reconstructPath(start_astar_node.id);
//             }
//             else{
//               aStarDataType end_point_node;
//               end_point_node.id = data_set_num_;
//               end_point_node.prev_id = start_astar_node.id;
//               end_point_node.pos = neighbor_center_point;
//               end_point_node.g_val = start_astar_node.g_val + start_point.distance(neighbor_center_point);
//               end_point_node.h_val = 0;
//               end_point_node.f_val = end_point_node.g_val;
//               data_set_vec_.push_back(end_point_node);
//               ++data_set_num_;
//               reconstructPath(end_point_node.id);
//             }
//             return true;
//           }
//           for (int k = 0; k < 6; ++k){
//             aStarDataType new_astar_node;
//             new_astar_node.prev_id = start_astar_node.id;
//             new_astar_node.ang_id = k;
//             new_astar_node.pos = neighbor_center_point;
//             new_astar_node.g_val = start_astar_node.g_val + start_point.distance(neighbor_center_point);
//             double ang_cost = 0;
//             /* if uav goes up in z axis */
//             if (k && 0x1 == 0)
//               ang_cost = new_astar_node.g_val;
//             else if (k != start_astar_node.ang_id)
//               ang_cost = neighbor_grid_gap * 0.707;
//             new_astar_node.g_val += ang_cost;
//             // Euclidean distance
//             new_astar_node.h_val = m_ara_star_rate * neighbor_center_point.distance(m_land_point);
//             // Manhattan distance
//             // new_astar_node.h_val = m_ara_star_rate *
//             //   (fabs(neighbor_center_point.x()-m_land_point.x())
//             //    + fabs(neighbor_center_point.y()-m_land_point.y())
//             //    + fabs(neighbor_center_point.z()-m_land_point.z()));
//             new_astar_node.f_val = new_astar_node.g_val + new_astar_node.h_val;
//             if (nodeInCloseSet(new_astar_node))
//               continue;
//             if (!nodeInOpenSet(new_astar_node))
//               {
//                 new_astar_node.id = data_set_num_;
//                 open_set_vec_.insert(getPosItearator(new_astar_node.f_val, 'o'), new_astar_node);
//                 data_set_vec_.push_back(new_astar_node);
//                 ++data_set_num_;
//               }
//           }
//         }
//       }
//     }
//   ROS_INFO("A Star Search failed!!");
//   return false;
// }

// void TruckServerNode::reconstructPath(int end_id)
// {
//   // From end to start
//   astar_path_vec_.push_back(m_land_point);
//   while(end_id != -1){
//     astar_path_vec_.push_back(data_set_vec_[end_id].pos);
//     end_id = data_set_vec_[end_id].prev_id;
//   }
//   astar_path_vec_.push_back(m_init_point);
// }

// std::vector<aStarDataType>::iterator TruckServerNode::getPosItearator(double f_val, char ch)
// {
//   if (ch == 'o'){
//     std::vector<aStarDataType>::iterator it = open_set_vec_.begin();
//     while (it != open_set_vec_.end()){
//       if (f_val > it->f_val) ++it;
//       else break;
//     }
//     return it;
//   }
//   else if (ch == 'c'){
//     std::vector<aStarDataType>::iterator it = close_set_vec_.begin();
//     while (it != close_set_vec_.end()){
//       if (f_val > it->f_val) ++it;
//       else break;
//     }
//     return it;
//   }
// }

// bool TruckServerNode::nodeInCloseSet(aStarDataType& node)
// {
//   std::vector<aStarDataType>::iterator it = close_set_vec_.begin();
//   std::vector<aStarDataType>::iterator it_pos;
//   while (it != close_set_vec_.end()){
//     if (node.pos == it->pos){
//       // If node in close set could be updated, move from close set to open set suitable position.
//       if (node.f_val < it->f_val){
//         node.id = it->id;
//         data_set_vec_[node.id] = node;
//         close_set_vec_.erase(it);
//         open_set_vec_.insert(getPosItearator(node.f_val, 'o'), node);
//       }
//       return true;
//     }
//     else ++it;
//   }
//   return false;
// }

// bool TruckServerNode::nodeInOpenSet(aStarDataType& node)
// {
//   std::vector<aStarDataType>::iterator it = open_set_vec_.begin();
//   while (it != open_set_vec_.end()){
//     if (node.pos == it->pos){
//       // if node in open set could be updated
//       if (node.f_val < it->f_val){
//         // Because f_val decreased, judge whether node's position in open set should be moved.
//         node.id = it->id;
//         data_set_vec_[node.id] = node;
//         open_set_vec_.erase(it);
//         open_set_vec_.insert(getPosItearator(node.f_val, 'o'), node);
//       }
//       return true;
//     }
//     else ++it;
//   }
//   return false;
// }


inline void TruckServerNode::vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32)
{
  point32.x = point3.x();
  point32.y = point3.y();
  point32.z = point3.z();
}

inline void TruckServerNode::vector3dConvertToPoint(Vector3d point3, geometry_msgs::Point& point)
{
  point.x = point3.x();
  point.y = point3.y();
  point.z = point3.z();
}

void TruckServerNode::truckOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_truck_odom = *msg;
  double cur_time = ros::Time().now().toSec();
  if (cur_time - m_vehicles_visualize_prev_time > m_vehicles_visualize_period_time){
    m_vehicles_visualize_prev_time = cur_time;
    vehicleCurrentPosVisualization(0);
    if (m_has_car_inner)
      vehicleCurrentPosVisualization(1); // 1 is small car
    if (m_has_car_outter)
      vehicleCurrentPosVisualization(2); // 2 is big car
    /* 1 is circle track, 2 has bridge, 3 has crossing car, 4 has bridge and crossing car */
    switch (m_route_id){
    case 2:
      obstacleCurrentPosVisualization(0); // truck
      break;
    case 3:
      break;
    case 4:
      break;
    default:
      break;
    }
    m_truck_ptr->publishTruckAll(ros::Time().now());
    m_truck_ptr->m_octree->clear();
  }
}

void TruckServerNode::carInnerOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_car_inner_odom = *msg;
}

void TruckServerNode::carOutterOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_car_outter_odom = *msg;
}
void TruckServerNode::aStarSearchGraphInit()
{
  m_seach_graph_connected_map.push_back(Vector3d(0, 0, 0));
  for (int i = 0; i < 3; ++i){
    Vector3d vec1 = Vector3d::Zero(3);
    vec1[i] = 1;
    m_seach_graph_connected_map.push_back(vec1);
    Vector3d vec2 = Vector3d::Zero(3);
    vec2[i] = -1;
    m_seach_graph_connected_map.push_back(vec2);
  }
  for (int i = 0; i < 3; ++i){
    Vector3d vec1 = Vector3d::Zero(3);
    vec1[i] = 1;
    for (int j = 0; j < 3; ++j){
      if (i != j){
        vec1[j] = 1;
        m_seach_graph_connected_map.push_back(vec1);
        vec1[j] = -1;
        m_seach_graph_connected_map.push_back(vec1);
      }
    }
    Vector3d vec2 = Vector3d::Zero(3);
    vec2[i] = -1;
    for (int j = 0; j < 3; ++j){
      if (i != j){
        vec2[j] = 1;
        m_seach_graph_connected_map.push_back(vec2);
        vec2[j] = -1;
        m_seach_graph_connected_map.push_back(vec2);
      }
    }
  }
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k){
        m_seach_graph_connected_map.push_back(Vector3d(2*i-1, 2*j-1, 2*k-1));
      }

}
