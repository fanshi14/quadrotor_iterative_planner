#ifndef OCTOMAP_TRUCK_SERVER_NODE_H_
#define OCTOMAP_TRUCK_SERVER_NODE_H_

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
#include <truck_server/QuadrotorCommand.h>
// Already defined in truck_server/QuadrotorCommand.h
// #include <bspline_ros/bsplineGenerate.h>

using namespace octomap_server;
using namespace vehicle_trajectory_base;

class TruckServerNode
{
public:
  ros::NodeHandle nh_;
  /* Subscriber */
  ros::Subscriber m_sub_lane_marker_flag;
  ros::Subscriber m_sub_truck_traj_param;
  ros::Subscriber m_sub_truck_odom;
  ros::Subscriber m_sub_uav_start_flag;
  ros::Subscriber m_sub_uav_odom;

  /* Publisher */
  ros::Publisher m_pub_uav_cmd;
  ros::Publisher m_pub_reconstructed_path_markers;


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

  /* truck information */
  nav_msgs::Odometry m_truck_odom;
  std::string m_truck_odom_sub_topic_name;
  VehicleTrajectoryBase m_truck_traj_base;
  TruckOctomapServer* m_truck_ptr;

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

  void onInit();

  /* callback function */
  void laneMarkerCallback(const std_msgs::Empty msg);
  void truckTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  void truckOdomCallback(const nav_msgs::OdometryConstPtr& msg);

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

void TruckServerNode::onInit()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("collision_detection", m_collision_detection_flag, false);
  private_nh.param("dji_mode", m_dji_mode, false);
  private_nh.param("debug_mode", m_debug_mode, true);
  private_nh.param("landing_mode", m_landing_mode, true);
  private_nh.param("resolution", m_octomap_res, 0.1);
  private_nh.param("tree_depth", m_octomap_tree_depth, 16);
  private_nh.param("route_id", m_route_id, 1);
  private_nh.param("spline_degree", m_spline_degree, 2);
  private_nh.param("vehicles_visualize_period_time", m_vehicles_visualize_period_time, 0.5);
  private_nh.param("global_planning_period_time", m_global_planning_period_default_time, 1.0);
  private_nh.param("target_height", m_target_height, 0.8);
  private_nh.param("uav_default_upbound_vel", m_uav_default_upbound_vel, 7.0);
  private_nh.param("uav_landing_time_xy_upbound", m_uav_landing_time_xy_upbound, 5.0);
  private_nh.param("uav_default_tracking_time", m_uav_default_tracking_time, 2.0);
  private_nh.param("truck_odom_sub_topic_name", m_truck_odom_sub_topic_name, (std::string)"/truck_odom");
  private_nh.param("spline_path_pub_topic_name", m_spline_path_pub_topic_name, (std::string)"spline_path");
  private_nh.param("uav_odom_sub_topic_name", m_uav_odom_sub_topic_name, (std::string)"/ground_truth/state");
  private_nh.param("uav_cmd_pub_topic_name", m_uav_cmd_pub_topic_name, (std::string)"/cmd_vel");
  private_nh.param("uav_direct_start_mode", m_uav_direct_start_mode, true);

  /* Init */
  m_uav.onInit();
  m_bspline_generator.onInit(m_spline_degree, true, m_spline_path_pub_topic_name);
  m_truck_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth);
  m_octomap_boarder_val = m_octomap_res * pow(2, m_octomap_tree_depth-1);
  m_vehicles_visualize_prev_time = ros::Time().now().toSec();
  m_vehicle_traj_recv_time = m_vehicles_visualize_prev_time;
  m_global_planning_period_time = m_global_planning_period_default_time;
  if (m_uav_direct_start_mode)
    m_uav.m_uav_state = 3; // in direct start mode, uav is ready to directly tracking.

  /* Subscriber */
  m_sub_lane_marker_flag = nh_.subscribe<std_msgs::Empty>("/lane_marker_flag", 1, &TruckServerNode::laneMarkerCallback, this);
  m_sub_truck_traj_param = nh_.subscribe<std_msgs::Float64MultiArray>("/truck_traj_param", 1, &TruckServerNode::truckTrajParamCallback, this);
  m_sub_truck_odom = nh_.subscribe<nav_msgs::Odometry>(m_truck_odom_sub_topic_name, 1, &TruckServerNode::truckOdomCallback, this);
  m_sub_uav_odom = nh_.subscribe<nav_msgs::Odometry>(m_uav_odom_sub_topic_name, 1, &TruckServerNode::uavOdomCallback, this);
  m_sub_uav_start_flag = nh_.subscribe<std_msgs::Empty>("/uav_start_flag", 1, &TruckServerNode::uavStartFlagCallback, this);

  /* Publisher */
  m_pub_reconstructed_path_markers = nh_.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  m_pub_uav_cmd  = nh_.advertise<geometry_msgs::Twist>(m_uav_cmd_pub_topic_name, 1);



  ROS_INFO("onInit finished");
}

void TruckServerNode::initIterativeSearching()
{
  /* Clear previous vector data, marker, octree data */
  if (!m_control_point_vec.empty()){
    controlPolygonDisplay(0);
    m_control_point_vec.clear();
  }
  if (!m_object_seg_ptr_vec.empty()){
    for (int i = 0; i < m_object_seg_ptr_vec.size(); ++i)
      m_object_seg_ptr_vec[i]->m_octree->clear();
    m_object_seg_ptr_vec.clear();
  }

  /* Initialize segment estimated landing time, which could be dynamically adjusted when meeting obstacles */
  // Curently we do not have data in truck odometry`s twist.
  // double truck_vel = sqrt(pow(m_truck_odom.twist.twist.linear.x, 2) +pow(m_truck_odom.twist.twist.linear.y, 2));
  double truck_vel = 4.17;
  double landing_time_xy = sqrt(pow(m_uav_odom.pose.pose.position.x-m_truck_odom.pose.pose.position.x, 2) + pow(m_uav_odom.pose.pose.position.y-m_truck_odom.pose.pose.position.y, 2)) / (m_uav_default_upbound_vel - truck_vel);
  if (m_landing_mode){
    double landing_time_z = (m_uav_odom.pose.pose.position.z - m_target_height) / 1.0;
    // if result is 5.1 s, we will use 6.0s as the landing time
    if (landing_time_z > landing_time_xy)
      m_landing_time = landing_time_z;
    else
      m_landing_time = landing_time_xy;
  }
  else{
    m_landing_time = landing_time_xy;
  }
  m_n_total_segments = (int)(m_landing_time + 0.49);
  if (m_landing_time > m_uav_landing_time_xy_upbound){
    m_landing_time = m_uav_landing_time_xy_upbound;
    m_n_segments = (int)(m_landing_time + 0.49);
  }
  else if (m_landing_time <= 2.0){
    m_landing_time = 2.0;
    m_n_total_segments = 2;
    m_n_segments = 2;
  }
  else
    m_n_segments = (int)(m_landing_time + 0.5);

  /* If landing time is too small, make planning time longer */
  if (m_landing_time < m_global_planning_period_time)
    m_global_planning_period_time = m_landing_time;

  /* Initialize segment default period time, which is 1.0s */
  m_segment_period_time = m_landing_time / m_n_segments;

  Vector3d control_pt_0(m_uav_odom.pose.pose.position.x, m_uav_odom.pose.pose.position.y, m_uav_odom.pose.pose.position.z);
  m_control_point_vec.push_back(control_pt_0);
  // Set second control point: P1 = P0 + v * Tp / 2
  while(1){
    if (m_collision_detection_flag){
      TruckOctomapServer* obstacle_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
      /* Add obstacle of inner and outter cars, and static obstacles like bridge */
      // updateObstacleOctomap(obstacle_ptr, 0);
      // todo: collision detection
      m_object_seg_ptr_vec.push_back(obstacle_ptr);
    }
    Vector3d control_pt_1 = control_pt_0 +
      Vector3d(m_uav_odom.twist.twist.linear.x*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.y*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.z*m_segment_period_time/2.0);
    m_control_point_vec.push_back(control_pt_1);
    break;

    // Vector3d temp_3d;
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
  Vector3d target_cur_pt;
  if (m_landing_mode)
    target_cur_pt = Vector3d(m_truck_odom.pose.pose.position.x, m_truck_odom.pose.pose.position.y, m_target_height);
  else
    target_cur_pt = Vector3d(m_truck_odom.pose.pose.position.x, m_truck_odom.pose.pose.position.y, (m_control_point_vec[0])[2]);
  for (int i = 1; i <= m_n_segments-2; ++i){
    Vector3d cur_control_pt;
    cur_control_pt = (m_control_point_vec[0] - target_cur_pt) / (m_n_total_segments-1) * (m_n_segments-1-i) + m_truck_traj_base.nOrderVehicleTrajectory(0, i*m_segment_period_time) + Vector3d(0.0, 0.0, target_cur_pt[2]);
    m_control_point_vec.push_back(cur_control_pt);
  }

  /* Set second control point: Pn-1 = Pn - v * Tp / 2 */
  /* Simple add last 2 control points without collision checking */
  // todo: check last 2 control points
  while(1){
    Vector3d control_pt_n;
    Vector3d control_pt_n_1;
  if (m_landing_mode)
    control_pt_n = m_truck_traj_base.nOrderVehicleTrajectory(0, m_landing_time) + Vector3d(0.0, 0.0, m_target_height);
  else
    control_pt_n = m_truck_traj_base.nOrderVehicleTrajectory(0, m_landing_time) + Vector3d(0.0, 0.0, (m_control_point_vec[0])[2]);
    // todo: adding landing speed. Currently z-axis value is 0 in last triangle.
    control_pt_n_1 = control_pt_n -
      m_truck_traj_base.nOrderVehicleTrajectory(1, m_landing_time) * m_segment_period_time/2.0;

    if (m_collision_detection_flag){
      TruckOctomapServer* obstacle_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
      /* Add obstacle of inner and outter cars, and static obstacles like bridge */
      // updateObstacleOctomap(obstacle_ptr, m_landing_time-m_segment_period_time);
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
  // uavRandomCheatOdom();

  initIterativeSearching();
  onIterativeSearching();

  /* test: specific control points test */
  //controlPtsRandomSet();

  /* publish control points */
  controlPolygonDisplay(1);

  /* Print the value of control points */
  // for (int i = 0; i < m_control_point_vec.size(); ++i){
  //   std::cout << "[Control pt] " << i << ": " << m_control_point_vec[i].x() << ", "
  //             << m_control_point_vec[i].y() << ", " << m_control_point_vec[i].z() << "\n";
  // }
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

void TruckServerNode::laneMarkerCallback(const std_msgs::Empty msg)
{
  m_truck_ptr->laneMarkerVisualization();
  // if (m_route_id == 3 || m_route_id == 4)
  //   m_truck_ptr->crossLaneMarkerVisualization();
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
}

void TruckServerNode::truckTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  double cur_time = ros::Time().now().toSec();
  if (cur_time - m_vehicle_traj_recv_time > m_global_planning_period_time){
    m_global_planning_period_time = m_global_planning_period_default_time;
    m_vehicle_traj_recv_time = cur_time;

    int traj_order = msg->layout.dim[0].size;
    std::vector<double> data;
    for (int i = 0; i < 2*traj_order+1; ++i)
      data.push_back(msg->data[i]);
    m_truck_traj_base.onInit(traj_order, data);

    /* run Iterative Searching */
    if (m_uav.m_uav_state == 3){
      runIterativeSearching();

      /* assign param to uav */
      m_uav.m_bspline_traj_ptr = &m_bspline_generator;
      m_uav.m_traj_updated = true;
      m_uav.m_traj_first_updated = true;
      m_uav.m_traj_updated_time = cur_time;
      m_uav.m_truck_traj = m_truck_traj_base;
    }
  }
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
    m_bspline_generator.bsplineParamInput(&control_polygon_points);
    /* Print knot velocity */
    m_bspline_generator.getDerive();
    // std::cout << "Print Knot info. \n" ;
    // for (int j = m_bspline_generator.m_deg; j < m_bspline_generator.m_n_knots - m_bspline_generator.m_deg; ++j){
    //   double knot_time = m_bspline_generator.m_knotpts[j];
    //   std::vector<double> knot_vel_vec = m_bspline_generator.evaluateDerive(knot_time);
    //   double knot_vel = sqrt(pow(knot_vel_vec[0],2) + pow(knot_vel_vec[1],2) + pow(knot_vel_vec[2],2));
    //   /* Debug output */
    //   if (knot_vel > 10.0)
    //     ROS_ERROR("!!!!!!!!!!!!!!!!!!!! Knot vel is too large!!!!");
    //   std::cout << "[Knot] " << j << ": " << knot_time << ", vel: " << knot_vel << "|| "
    //             << knot_vel_vec[0] << ", "<< knot_vel_vec[1] << ", "<< knot_vel_vec[2] << "\n";
    // }
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

  m_pub_reconstructed_path_markers.publish(path_markers);
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
  /* uav */
  m_uav.getTruckOdom(msg);

  m_truck_odom = *msg;
  double cur_time = ros::Time().now().toSec();
  if (cur_time - m_vehicles_visualize_prev_time > m_vehicles_visualize_period_time){
    m_vehicles_visualize_prev_time = cur_time;
    vehicleCurrentPosVisualization(0);
    m_truck_ptr->publishTruckAll(ros::Time().now());
    m_truck_ptr->m_octree->clear();
  }
}

void TruckServerNode::uavStartFlagCallback(const std_msgs::Empty msg)
{
  m_uav.m_uav_state = 1;
}

void TruckServerNode::uavOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_uav_odom = *msg;
  /* state: 0, still; 1, taking off; 2, ready to move; 3, start to move; 4, landing finishes */
  m_uav.getUavOdom(msg);
  if (m_uav.m_uav_state == 1){
    m_uav.uavMovingToPresetHeight(10.0);
    m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
  }
  else if (m_uav.m_uav_state == 2){
    if (m_uav.uavTruckHorizonDistance() < 6.0){
      m_uav.m_uav_state = 3;
    }
  }
  else if (m_uav.m_uav_state == 3){
    /* In case traj not being calculated before state changes to 3 */
    if (m_uav.m_traj_first_updated){
      m_uav.trackTrajectory();
      m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
    }
  }
}

#endif
