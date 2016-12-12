
#ifndef OCTOMAP_SERVER_TRUCK_H_
#define OCTOMAP_SERVER_TRUCK_H_

//#include "/home/shi/ros/shi_catkin_ws/src/octomap_mapping/octomap_server/include/octomap_server/OctomapServer.h"
#include <octomap_server/OctomapServer.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <math.h>
#include <vector>

using namespace octomap_server;
using namespace octomap;
using namespace octomath;
class TruckOctomapServer: public OctomapServer {
public:
  TruckOctomapServer();
  virtual ~TruckOctomapServer();
  void init_param();
  void WriteTruckOctree(Pose6D rot_mat);
  void publishTruckFullOctoMap(const ros::Time& rostime);
  void publishTruckAll(const ros::Time& rostime);
  //std::vector<int> roof, base, cargo;
  int roof[3], base[3], cargo[3];
  float step_value;
};

#endif
