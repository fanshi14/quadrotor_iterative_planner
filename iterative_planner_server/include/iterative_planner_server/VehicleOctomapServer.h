
#ifndef OCTOMAP_SERVER_VEHICLE_H_
#define OCTOMAP_SERVER_VEHICLE_H_

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
class VehicleOctomapServer: public OctomapServer {
public:
  VehicleOctomapServer(double resolution, int tree_depth);
  VehicleOctomapServer(double resolution, int tree_depth, bool is_publish_topic);
  virtual ~VehicleOctomapServer();
  float m_step_value;
  ros::Publisher m_pub_lane_marker;
  ros::Publisher m_pub_cross_lane_marker;
  int m_route_id;
  float m_route_radius;
  std::string m_vehicle_odom_sub_topic_name;

  void init_param();
  void WriteVehicleOctree(int type, Pose6D rot_mat);
  void WriteUavSafeBorderOctree(int type, Pose6D rot_mat);
  void WriteObstacleOctree(int type, Pose6D rot_mat);
  void publishVehicleFullOctoMap(const ros::Time& rostime);
  void publishVehicleAll(const ros::Time& rostime);
  void laneMarkerVisualization();
  void crossLaneMarkerVisualization();
};

#endif
