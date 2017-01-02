#include <ros/ros.h>
#include <truck_server/TruckOctomapServer.h>
#include <unistd.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace octomap_server;

struct aStarDataType
{
  point3d pos;
  double g_val;
  double h_val;
  double f_val;
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
  ros::Subscriber sub_truck_octomap_flag_;
  ros::Subscriber sub_point_depth_query_;
  ros::Subscriber sub_astar_query_;
  ros::Publisher pub_point_octocube_;
  ros::Publisher pub_reconstructed_path_markers_;

  std::vector<aStarDataType> open_set_vec_, close_set_vec_, data_set_vec_;
  int data_set_num_;
  std::vector<point3d> astar_path_vec_;
  point3d init_point, land_point;

  TruckOctomapServer truck_;
  void pointOccupiedQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void truckOctomapCallback(const std_msgs::Empty msg);
  void astarPathQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void onInit();
  // return true if the grid is free
  bool getGridCenter(point3d query_point, point3d& center_point, int depth);
  void aStarSearchInit();
  bool aStarSearch();
  std::vector<aStarDataType>::iterator getPosItearator(double f_val, char ch);
  bool nodeInCloseSet(aStarDataType& node);
  bool nodeInOpenSet(aStarDataType& node);
  void reconstructPath(int end_id);
  void reconstructedPathDisplay(int mode); // mode: 1, add display; -1, delete display
};

void TruckServerNode::onInit()
{
  sub_point_occupied_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_occupied", 10, &TruckServerNode::pointOccupiedQueryCallback, this);
  sub_truck_octomap_flag_ = nh_.subscribe<std_msgs::Empty>("/truck_octomap_flag", 1, &TruckServerNode::truckOctomapCallback, this);
  sub_point_depth_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_depth", 1, &TruckServerNode::pointDepthQueryCallback, this);
  sub_astar_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_astar_path", 1, &TruckServerNode::astarPathQueryCallback, this);

  pub_point_octocube_ = nh_.advertise<visualization_msgs::Marker>("octo_cube_marker", 1);
  pub_reconstructed_path_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);

  std::cout << "onInit finished.\n";
  ROS_INFO("onInit");
}

void TruckServerNode::pointOccupiedQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("Query");
  std::cout << "query comes.\n";
  point3d query(msg->x, msg->y, msg->z);
  OcTreeNode* result = truck_.m_octree->search (query);
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

void TruckServerNode::truckOctomapCallback(const std_msgs::Empty msg)
{
  double rot_deg = 0;
  double rot_ang = rot_deg * 3.1415926 / 360.0;
  // x,y,z,r,p,y
  truck_.WriteVehicleOctree(0, Pose6D(0.0f, 0.0f, 0.0f, 0.0, 0.0, rot_ang));
  truck_.WriteVehicleOctree(1, Pose6D(0.0f, 3.5f/cos(rot_ang), 0.0f, 0.0, 0.0, rot_ang));
  truck_.WriteVehicleOctree(2, Pose6D(0.0f, -3.5f/cos(rot_ang), 0.0f, 0.0, 0.0, rot_ang));
  truck_.laneMarkerVisualization();

  // truck_.WriteUavSafeBorderOctree(0, Pose6D(0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0));
  // truck_.WriteUavSafeBorderOctree(1, Pose6D(0.0f, 3.5f, 0.0f, 0.0, 0.0, 0.0));
  // truck_.WriteUavSafeBorderOctree(2, Pose6D(0.0f, -3.5f, 0.0f, 0.0, 0.0, 0.0));

  truck_.publishTruckAll(ros::Time().now());
  usleep(1000000);
}

void TruckServerNode::astarPathQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("AstarPathQuery");
  // If alredy have diplay, delete previous display first.
  if (astar_path_vec_.size() > 0)
    reconstructedPathDisplay(-1);
  init_point = point3d(msg->x, msg->y, msg->z);
  land_point = point3d(-1, 0, 1.3);
  aStarSearch();
  ROS_INFO("Search finished");
  reconstructedPathDisplay(1);
  ROS_INFO("Display finished");
}

void TruckServerNode::pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("QueryDepth");
  point3d query(msg->x, msg->y, msg->z);
  std::cout << msg->x <<' ' << msg->y << ' ' << msg->z <<'\n';
  // Depth from 1 to 16
  int cur_depth;
  OcTreeNode* result = truck_.m_octree->searchReturnDepth (query, 0, cur_depth);
  double cube_size = truck_.m_octree->resolution * pow(2, truck_.m_octree->tree_depth-cur_depth);
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
  for (int i = points_num-1; i >=0 ; --i){
    int cur_depth;
    truck_.m_octree->searchReturnDepth(astar_path_vec_[i], 0, cur_depth);
    double cube_size = truck_.m_octree->resolution * pow(2, truck_.m_octree->tree_depth-cur_depth);
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
}

bool TruckServerNode::getGridCenter(point3d query_point, point3d& center_point, int depth)
{
  bool isGridFree = true;
  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      OcTreeNode* result = truck_.m_octree->searchReturnDepth(query_point, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }
  key_type key_x = truck_.m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = truck_.m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = truck_.m_octree->coordToKey(query_point.z(), depth);
  double center_x = truck_.m_octree->keyToCoord(key_x, depth);
  double center_y = truck_.m_octree->keyToCoord(key_y, depth);
  double center_z = truck_.m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}

void TruckServerNode::aStarSearchInit()
{
  data_set_num_ = 0;
  data_set_vec_.clear();
  open_set_vec_.clear();
  close_set_vec_.clear();
  astar_path_vec_.clear();

  point3d start_point;
  getGridCenter(init_point, start_point, -1);
  int start_node_depth;
  truck_.m_octree->searchReturnDepth(start_point, 0, start_node_depth);
  double start_grid_size = truck_.m_octree->resolution * pow(2, truck_.m_octree->tree_depth-start_node_depth);
  double neighbor_grid_gap = start_grid_size/2.0 + truck_.m_octree->resolution/2.0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z)
        {
          point3d neighbor_center_point;
          if (getGridCenter((start_point + point3d(neighbor_grid_gap*x,
                                                   neighbor_grid_gap*y,
                                                   neighbor_grid_gap*z)),
                            neighbor_center_point, -1))
            {
              if (neighbor_center_point.z() < 0)
                continue;
              aStarDataType new_astar_node;
              new_astar_node.prev_id = -1;
              new_astar_node.id = data_set_num_;
              new_astar_node.pos = neighbor_center_point;
              new_astar_node.g_val = init_point.distance(neighbor_center_point);
              // Euclidean distance
              //new_astar_node.h_val = neighbor_center_point.distance(land_point);
              // Manhattan distance
              new_astar_node.h_val = fabs(neighbor_center_point.x()-land_point.x())
                + fabs(neighbor_center_point.y()-land_point.y())
                + fabs(neighbor_center_point.z()-land_point.z());
              new_astar_node.f_val = new_astar_node.g_val + new_astar_node.h_val;
              open_set_vec_.insert(getPosItearator(new_astar_node.f_val, 'o'), new_astar_node);
              data_set_vec_.push_back(new_astar_node);
              ++data_set_num_;
              //test
              //astar_path_vec_.push_back(neighbor_center_point);
            }
        }
}


bool TruckServerNode::aStarSearch()
{
  aStarSearchInit();
  point3d end_point;
  getGridCenter(land_point, end_point, -1);

  while (!open_set_vec_.empty())
    {
      //std::cout << "open set size: " << open_set_vec_.size() << '\n';
      // Pop first element in open set, and push into close set
      aStarDataType start_astar_node = open_set_vec_[0];
      point3d start_point = start_astar_node.pos;
      // Judge whether reach the goal point
      // TODO: better end conditions is needed
      if (start_point == end_point || start_point.distance(land_point) < 0.2*sqrt(3))
        {
          ROS_INFO("Reach the goal point.");
          reconstructPath(start_astar_node.id);
          return true;
        }
      close_set_vec_.insert(getPosItearator(start_astar_node.f_val, 'c'), start_astar_node);
      open_set_vec_.erase(open_set_vec_.begin());

      int start_octree_node_depth;
      truck_.m_octree->searchReturnDepth(start_point, 0, start_octree_node_depth);
      double start_grid_size = truck_.m_octree->resolution * pow(2, truck_.m_octree->tree_depth-start_octree_node_depth);
      double neighbor_grid_gap = start_grid_size/2.0 + truck_.m_octree->resolution/2.0;
      for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
          for (int z = -1; z <= 1; ++z)
            {
              if (x == 0 && y == 0 && z == 0)
                continue;
              point3d neighbor_center_point;
              if (getGridCenter((start_point + point3d(neighbor_grid_gap*x,
                                                       neighbor_grid_gap*y,
                                                       neighbor_grid_gap*z)),
                                neighbor_center_point, -1))
                {
                  if (neighbor_center_point.z() < 0)
                    continue;
                  aStarDataType new_astar_node;
                  new_astar_node.prev_id = start_astar_node.id;
                  new_astar_node.pos = neighbor_center_point;
                  new_astar_node.g_val = start_astar_node.g_val + start_point.distance(neighbor_center_point);
                  // Euclidean distance
                  //new_astar_node.h_val = neighbor_center_point.distance(land_point);
                  // Manhattan distance
                  new_astar_node.h_val = fabs(neighbor_center_point.x()-land_point.x())
                    + fabs(neighbor_center_point.y()-land_point.y())
                    + fabs(neighbor_center_point.z()-land_point.z());
                  new_astar_node.f_val = new_astar_node.g_val + new_astar_node.h_val;
                  if (nodeInCloseSet(new_astar_node))
                    continue;
                  if (!nodeInOpenSet(new_astar_node))
                    {
                      new_astar_node.id = data_set_num_;
                      open_set_vec_.insert(getPosItearator(new_astar_node.f_val, 'o'), new_astar_node);
                      data_set_vec_.push_back(new_astar_node);
                      ++data_set_num_;
                    }
                }
            }
    }
  ROS_INFO("A Star Search failed!!");
  return false;
}

void TruckServerNode::reconstructPath(int end_id)
{
  // From end to start
  astar_path_vec_.push_back(land_point);
  while(end_id != -1){
    astar_path_vec_.push_back(data_set_vec_[end_id].pos);
    end_id = data_set_vec_[end_id].prev_id;
  }
  astar_path_vec_.push_back(init_point);
}

std::vector<aStarDataType>::iterator TruckServerNode::getPosItearator(double f_val, char ch)
{
  if (ch == 'o'){
    std::vector<aStarDataType>::iterator it = open_set_vec_.begin();
    while (it != open_set_vec_.end()){
      if (f_val > it->f_val) ++it;
      else break;
    }
    return it;
  }
  else if (ch == 'c'){
    std::vector<aStarDataType>::iterator it = close_set_vec_.begin();
    while (it != close_set_vec_.end()){
      if (f_val > it->f_val) ++it;
      else break;
    }
    return it;
  }
}

bool TruckServerNode::nodeInCloseSet(aStarDataType& node)
{
  std::vector<aStarDataType>::iterator it = close_set_vec_.begin();
  std::vector<aStarDataType>::iterator it_pos;
  while (it != close_set_vec_.end()){
    if (node.pos == it->pos){
      // If node in close set could be updated, move from close set to open set suitable position.
      if (node.f_val < it->f_val){
        node.id = it->id;
        data_set_vec_[node.id] = node;
        close_set_vec_.erase(it);
        open_set_vec_.insert(getPosItearator(node.f_val, 'o'), node);
      }
      return true;
    }
    else ++it;
  }
  return false;
}

bool TruckServerNode::nodeInOpenSet(aStarDataType& node)
{
  std::vector<aStarDataType>::iterator it = open_set_vec_.begin();
  while (it != open_set_vec_.end()){
    if (node.pos == it->pos){
      // if node in open set could be updated
      if (node.f_val < it->f_val){
        // Because f_val decreased, judge whether node's position in open set should be moved.
        node.id = it->id;
        data_set_vec_[node.id] = node;
        open_set_vec_.erase(it);
        open_set_vec_.insert(getPosItearator(node.f_val, 'o'), node);
      }
      return true;
    }
    else ++it;
  }
  return false;
}
