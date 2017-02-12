
#ifndef OCTOMAP_SERVER_TRUCK_H_
#define OCTOMAP_SERVER_TRUCK_H_

//#include "/home/shi/ros/shi_catkin_ws/src/octomap_mapping/octomap_server/include/octomap_server/OctomapServer.h"
#include <octomap_server/OctomapServer.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>

using namespace octomap_server;
using namespace octomap;
using namespace octomath;
class TruckOctomapServer: public OctomapServer {
public:
  TruckOctomapServer(double resolution, int tree_depth);
  virtual ~TruckOctomapServer();
  float m_step_value;
  ros::Publisher m_pub_lane_marker;
  std::string m_route_name;
  float m_route_radius;
  std::string m_truck_odom_sub_topic_name;

  void init_param();
  void WriteVehicleOctree(int type, Pose6D rot_mat);
  void WriteUavSafeBorderOctree(int type, Pose6D rot_mat);
  void WriteObstacleOctree(int type, Pose6D rot_mat);
  void publishTruckFullOctoMap(const ros::Time& rostime);
  void publishTruckAll(const ros::Time& rostime);
  void laneMarkerVisualization();
};

#endif
