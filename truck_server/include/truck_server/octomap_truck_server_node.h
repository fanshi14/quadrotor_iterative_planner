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
