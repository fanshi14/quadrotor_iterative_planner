#include <ros/ros.h>
#include <truck_server/TruckOctomapServer.h>
#include <unistd.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include <iostream>

using namespace octomap_server;

class TruckServerNode
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_point_occupied_query_;
  ros::Subscriber sub_truck_octomap_flag_;
  ros::Subscriber sub_point_depth_query_;
  ros::Publisher pub_point_octocube_;

  TruckOctomapServer truck_;
  void pointOccupiedQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void truckOctomapCallback(const std_msgs::Empty msg);
  void onInit();
};

void TruckServerNode::onInit()
{
  sub_point_occupied_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_occupied", 10, &TruckServerNode::pointOccupiedQueryCallback, this);
  sub_truck_octomap_flag_ = nh_.subscribe<std_msgs::Empty>("/truck_octomap_flag", 1, &TruckServerNode::truckOctomapCallback, this);
  sub_point_depth_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point_depth", 1, &TruckServerNode::pointDepthQueryCallback, this);

  pub_point_octocube_ = nh_.advertise<visualization_msgs::Marker>("octo_cube_marker", 1);

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
      std::cout << "Unknown point" << query << std::endl;
    }
  else
    {
      std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
      std::cout << "Logodds at " << query << ":\t " << result->getLogOdds() << std::endl << std::endl;
    }
}

void TruckServerNode::truckOctomapCallback(const std_msgs::Empty msg)
{
  // x,y,z,r,p,y
  truck_.WriteVehicleOctree(0, Pose6D(0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0));
  truck_.WriteVehicleOctree(1, Pose6D(0.0f, 3.5f, 0.0f, 0.0, 0.0, 0.0));
  truck_.WriteVehicleOctree(2, Pose6D(0.0f, -3.5f, 0.0f, 0.0, 0.0, 0.0));
  truck_.laneMarkerVisualization();

  //truck_.m_octree->updateNode(point3d(-3276.8f, -3276.8f, -3276.8f), true);
 //truck.publishTruckFullOctoMap(ros::Time().now());
 truck_.publishTruckAll(ros::Time().now());
 usleep(1000000);
}

void TruckServerNode::pointDepthQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("QueryDepth");
  point3d query(msg->x, msg->y, msg->z);
  std::cout << msg->x <<' ' << msg->y << ' ' << msg->z <<'\n';
  OcTreeNode* result = truck_.m_octree->search (query, 0);
  // Depth from 1 to 16
  int cur_depth = result->depth;
  double cube_size = truck_.m_octree->resolution * pow(2, truck_.m_octree->tree_depth-cur_depth);
  std::cout << "Point is " << cur_depth << "\n";
  key_type key_x = truck_.m_octree->coordToKey(msg->x, cur_depth);
  key_type key_y = truck_.m_octree->coordToKey(msg->y, cur_depth);
  key_type key_z = truck_.m_octree->coordToKey(msg->z, cur_depth);
  double center_x = truck_.m_octree->keyToCoord(key_x, cur_depth);
  double center_y = truck_.m_octree->keyToCoord(key_y, cur_depth);
  double center_z = truck_.m_octree->keyToCoord(key_z, cur_depth);

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

  //
  octo_cube_marker.pose.position.x = center_x;
  octo_cube_marker.pose.position.y = center_y;
  octo_cube_marker.pose.position.z = center_z;
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

  //pub_point_octocube_ = nh.advertise<visualization_msgs::Marker>("lane_marker", 10);
  pub_point_octocube_.publish(query_point_marker);
  //sleep(1.5);
  pub_point_octocube_.publish(octo_cube_marker);

}
