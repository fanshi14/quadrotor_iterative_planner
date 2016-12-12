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
  ros::Subscriber sub_point_query_;
  ros::Subscriber sub_truck_octomap_flag_;
  TruckOctomapServer truck_;
  void pointQueryCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void truckOctomapCallback(const std_msgs::Empty msg);
  void onInit();
};

void TruckServerNode::onInit()
{
  sub_point_query_ = nh_.subscribe<geometry_msgs::Vector3>("/query_point", 10, &TruckServerNode::pointQueryCallback, this);
  sub_truck_octomap_flag_ = nh_.subscribe<std_msgs::Empty>("/truck_octomap_flag", 1, &TruckServerNode::truckOctomapCallback, this);
  std::cout << "onInit finished.\n";
  ROS_INFO("onInit");
}

void TruckServerNode::pointQueryCallback(const geometry_msgs::Vector3ConstPtr& msg){
  ROS_INFO("Query");
  std::cout << "query comes.\n";
  point3d query(msg->x, msg->y, msg->z);
  OcTreeNode* result = truck_.m_octree->search (query);
  std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
}

void TruckServerNode::truckOctomapCallback(const std_msgs::Empty msg)
{
 truck_.WriteTruckOctree(Pose6D(0,0,0,0,0,-M_PI/4));
 //truck.publishTruckFullOctoMap(ros::Time().now());
 truck_.publishTruckAll(ros::Time().now());
 usleep(1000000);
}
