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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_trajectory/VehicleTrajectoryBase.h>

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
  ros::Publisher pub_path_grid_points_;

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

  int m_car_inner_type;
  int m_car_outter_type;

  VehicleTrajectoryBase m_truck_traj_base;
  VehicleTrajectoryBase m_car_inner_traj_base;
  VehicleTrajectoryBase m_car_outter_traj_base;

  TruckOctomapServer* m_truck_ptr;
  std::vector<TruckOctomapServer*> m_object_seg_ptr_vec;

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

  void onInit();
  void aStarSearchGraphInit();
  // return true if the grid is free
  bool isInsideBoarder(point3d query_point);
  bool getGridCenter(point3d query_point, point3d& center_point, int depth);
  bool aStarSearchInit();
  bool aStarSearch();
  std::vector<aStarDataType>::iterator getPosItearator(double f_val, char ch);
  bool nodeInCloseSet(aStarDataType& node);
  bool nodeInOpenSet(aStarDataType& node);
  void reconstructPath(int end_id);
  void reconstructedPathDisplay(int mode); // mode: 1, add display; -1, delete display
  inline void point3dConvertToPoint32(point3d point3, geometry_msgs::Point32& point32);
  // test
  void runAstarTest();
};

void TruckServerNode::onInit()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("resolution", m_octomap_res, 0.1);
  private_nh.param("tree_depth", m_octomap_tree_depth, 16);
  private_nh.param("has_car_inner", m_has_car_inner, true);
  private_nh.param("has_car_outter", m_has_car_outter, true);
  private_nh.param("vehilce_inner_type", m_car_inner_type, 1);
  private_nh.param("vehilce_outter_type", m_car_outter_type, 2);
  private_nh.param("octomap_update_rate", m_octomap_update_feq, 1);
  private_nh.param("ARA_rate", m_ara_star_rate, 1.0);
  private_nh.param("graph_connected_mode", m_graph_connected_mode, 27);
  private_nh.param("vehicles_visualize_period_time", m_vehicles_visualize_period_time, 0.5);
  private_nh.param("global_planning_period_time", m_global_planning_period_time, 1.0);


  /* 0 is only visualize vehicles current position, 1 is visualize their future position with connected octomap. */
  private_nh.param("vehicle_octomap_visualize_mode", m_vehicle_octomap_visualize_mode, 0);

  m_truck_ptr = new TruckOctomapServer(m_octomap_res, m_octomap_tree_depth);

  m_octomap_update_feq_cnt = 0;
  m_octo_publish_all_cnt = 0;
  m_octomap_boarder_val = m_octomap_res * pow(2, m_octomap_tree_depth-1);

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
  pub_path_grid_points_  = nh_.advertise<geometry_msgs::PolygonStamped>("path_gird_points", 1);

  std::cout << "onInit finished.\n";
  ROS_INFO("onInit");

  m_vehicles_visualize_prev_time = ros::Time().now().toSec();
  m_vehicle_traj_recv_time = m_vehicles_visualize_prev_time;

  // todo: get from launch
  spline_res = 0.2f;
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
    if (m_has_car_inner){
      int traj_order1 = m_car_inner_traj_msg.layout.dim[0].size;
      std::vector<double> data1;
      for (int i = 0; i < 2*traj_order1+1; ++i)
        data1.push_back(m_car_inner_traj_msg.data[i]);
      m_car_inner_traj_base.onInit(traj_order1, data1);
    }

    // Load car outter's trajectory paramaters
    if (m_has_car_outter){
      int traj_order1 = m_car_outter_traj_msg.layout.dim[0].size;
      std::vector<double> data1;
      for (int i = 0; i < 2*traj_order1+1; ++i)
        data1.push_back(m_car_outter_traj_msg.data[i]);
      m_car_outter_traj_base.onInit(traj_order1, data1);
    }
  }
}


void TruckServerNode::carInnerTrajParamCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  m_car_inner_traj_msg = *msg;

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

void TruckServerNode::reconstructedPathDisplay(int mode){
  int points_num = astar_path_vec_.size();
  std::cout << "[Display] points number: " << points_num << "\n";
  int id_cnt = 0;
  visualization_msgs::MarkerArray path_markers;
  visualization_msgs::Marker center_point_marker, octo_cube_marker, arrow_marker;
  octo_cube_marker.ns = center_point_marker.ns = arrow_marker.ns = "pathcubes";
  octo_cube_marker.header.frame_id = center_point_marker.header.frame_id = arrow_marker.header.frame_id = std::string("/world");
  octo_cube_marker.header.stamp = center_point_marker.header.stamp = arrow_marker.header.stamp = ros::Time().now();
  if (mode == 1)
    octo_cube_marker.action = center_point_marker.action = arrow_marker.action = visualization_msgs::Marker::ADD;
  else
    octo_cube_marker.action = center_point_marker.action = arrow_marker.action = visualization_msgs::Marker::DELETE;
  octo_cube_marker.type = visualization_msgs::Marker::CUBE;
  center_point_marker.type = visualization_msgs::Marker::SPHERE;
  arrow_marker.type = visualization_msgs::Marker::ARROW;

  geometry_msgs::PolygonStamped path_grid_points;
  path_grid_points.header = octo_cube_marker.header;
  geometry_msgs::Point32 grid_point, time_point;
  time_point.x = 0.0f; time_point.y = 0.0f; time_point.z = 0.0f;
  path_grid_points.polygon.points.push_back(time_point);
  point3dConvertToPoint32(m_init_point, grid_point);
  path_grid_points.polygon.points.push_back(grid_point);

  for (int i = points_num-1; i >=0 ; --i){
    int cur_depth;
    m_truck_ptr->m_octree->searchReturnDepth(astar_path_vec_[i], 0, cur_depth);
    double cube_size = m_truck_ptr->m_octree->resolution * pow(2, m_truck_ptr->m_octree->tree_depth-cur_depth);
    // if (result == NULL)
    //   std::cout << "Unknown/free region.\n";
    // else
    //   std::cout << "Prob is  " << result->getOccupancy() << "\n";

    if (i >= 1){
      arrow_marker.id = id_cnt;
      ++id_cnt;
      geometry_msgs::Point s_pt, e_pt;
      s_pt.x = astar_path_vec_[i-1].x();
      s_pt.y = astar_path_vec_[i-1].y();
      s_pt.z = astar_path_vec_[i-1].z();
      e_pt.x = astar_path_vec_[i].x();
      e_pt.y = astar_path_vec_[i].y();
      e_pt.z = astar_path_vec_[i].z();
      arrow_marker.points.clear();
      arrow_marker.points.push_back(s_pt);
      arrow_marker.points.push_back(e_pt);
      //scale.x is the shaft diameter, and scale.y is the head diameter. scale.z is the head length.
      arrow_marker.scale.x = 0.05;
      arrow_marker.scale.y = 0.2;
      arrow_marker.scale.z = 0.07;
      arrow_marker.color.a = 1;
      arrow_marker.color.r = 0.0f;
      arrow_marker.color.g = 0.0f;
      arrow_marker.color.b = 1.0f;
      path_markers.markers.push_back(arrow_marker);

      time_point.x += spline_res * int(astar_path_vec_[i].distance(astar_path_vec_[i-1]) / spline_res);
      path_grid_points.polygon.points.push_back(time_point);
      point3dConvertToPoint32(astar_path_vec_[i-1], grid_point);
      path_grid_points.polygon.points.push_back(grid_point);
    }

    center_point_marker.id = id_cnt;
    ++id_cnt;
    center_point_marker.pose.position.x = astar_path_vec_[i].x();
    center_point_marker.pose.position.y = astar_path_vec_[i].y();
    center_point_marker.pose.position.z = astar_path_vec_[i].z();
    center_point_marker.pose.orientation.x = 0.0;
    center_point_marker.pose.orientation.y = 0.0;
    center_point_marker.pose.orientation.z = 0.0;
    center_point_marker.pose.orientation.w = 1.0;
    if (i == 0 || i == points_num-1){
      center_point_marker.scale.x = 0.3;
      center_point_marker.scale.y = 0.3;
      center_point_marker.scale.z = 0.3;
      center_point_marker.color.a = 1;
      center_point_marker.color.r = 1.0f;
      center_point_marker.color.g = 0.0f;
      center_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(center_point_marker);
      continue;
    }
    else{
      center_point_marker.scale.x = 0.05;
      center_point_marker.scale.y = 0.05;
      center_point_marker.scale.z = 0.05;
      center_point_marker.color.a = 1;
      center_point_marker.color.r = 1.0f;
      center_point_marker.color.g = 0.0f;
      center_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(center_point_marker);
    }

    octo_cube_marker.id = id_cnt;
    ++id_cnt;
    octo_cube_marker.pose.position.x = astar_path_vec_[i].x();
    octo_cube_marker.pose.position.y = astar_path_vec_[i].y();
    octo_cube_marker.pose.position.z = astar_path_vec_[i].z();
    octo_cube_marker.pose.orientation.x = 0.0;
    octo_cube_marker.pose.orientation.y = 0.0;
    octo_cube_marker.pose.orientation.z = 0.0;
    octo_cube_marker.pose.orientation.w = 1.0;
    octo_cube_marker.scale.x = cube_size;
    octo_cube_marker.scale.y = cube_size;
    octo_cube_marker.scale.z = cube_size;
    octo_cube_marker.color.a = 0.5;
    octo_cube_marker.color.r = 0.0f;
    octo_cube_marker.color.g = 1.0f;
    octo_cube_marker.color.b = 0.0f;
    path_markers.markers.push_back(octo_cube_marker);
  }
  pub_reconstructed_path_markers_.publish(path_markers);
  if (mode == 1)
    pub_path_grid_points_.publish(path_grid_points);
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


inline void TruckServerNode::point3dConvertToPoint32(point3d point3, geometry_msgs::Point32& point32)
{
  point32.x = point3.x();
  point32.y = point3.y();
  point32.z = point3.z();
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
