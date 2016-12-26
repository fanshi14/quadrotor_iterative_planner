
#include <truck_server/TruckOctomapServer.h>
#include <string>

using namespace octomap;
using namespace octomath;
using namespace octomap_server;

TruckOctomapServer::TruckOctomapServer() :
  OctomapServer()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("resolution", m_res, 0.1);
  private_nh.param("frame_id", m_worldFrameId, (std::string)"/map");

  init_param();
}

TruckOctomapServer::~TruckOctomapServer() {}

void TruckOctomapServer::init_param()
{
  m_octree->setResolution(m_res);

  step_value = (float)m_res / 2.0f;

  m_octree->enableChangeDetection(true);

  pub_lane_marker = this->m_nh.advertise<visualization_msgs::Marker>("lane_marker", 10);

  printf("Layers: %d %d\n", m_octree->tree_depth, (int)m_octree->tree_size);
}


void TruckOctomapServer::publishTruckFullOctoMap(const ros::Time& rostime)
{
  OctomapServer *b = this;
  this->publishFullOctoMap(rostime);
}

void TruckOctomapServer::publishTruckAll(const ros::Time& rostime)
{
  OctomapServer *b = this;
  this->publishAll(rostime);
}

void TruckOctomapServer::WriteVehicleOctree(int type, Pose6D rot_mat)
{
  int roof[3], base[3], cargo[3];
  float roof_offset[3], base_offset[3], cargo_offset[3];
  // truck for challenge
  if (type == 0)
    {
      roof[0] = (int)round(1.0/m_res); roof[1] = (int)round(1.5/m_res); roof[2] = (int)round(1.0/m_res);
      base[0] = (int)round(2.5/m_res); base[1] = (int)round(1.5/m_res); base[2] = (int)round(1.0/m_res);
      cargo[0] = 0; cargo[1] = 0; cargo[2] = 0;
      roof_offset[0] = 0.75f; roof_offset[1] = 0.0f; roof_offset[2] = 1.5f;
      base_offset[0] = 0.0f; base_offset[1] = 0.0f; base_offset[2] = 0.5f;
      cargo_offset[0] = 0.0f; cargo_offset[1] = 0.0f; cargo_offset[2] = 0.0f;
    }
  // sedan vehicle
  else if (type == 1)
    {
      roof[0] = (int)round(2.4/m_res); roof[1] = (int)round(2.0/m_res); roof[2] = (int)round(0.8/m_res);
      base[0] = (int)round(4.8/m_res); base[1] = (int)round(2.0/m_res); base[2] = (int)round(0.8/m_res);
      cargo[0] = 0; cargo[1] = 0; cargo[2] = 0;
      roof_offset[0] = 0.0f; roof_offset[1] = 0.0f; roof_offset[2] = 1.2f;
      base_offset[0] = 0.0f; base_offset[1] = 0.0f; base_offset[2] = 0.4f;
      cargo_offset[0] = 0.0f; cargo_offset[1] = 0.0f; cargo_offset[2] = 0.0f;
    }
  // big truck
  else if (type == 2)
    {
      roof[0] = (int)round(2.0/m_res); roof[1] = (int)round(2.4/m_res); roof[2] = (int)round(1.2/m_res);
      base[0] = (int)round(10.0/m_res); base[1] = (int)round(2.4/m_res); base[2] = (int)round(1.4/m_res);
      cargo[0] = (int)round(7.4/m_res); cargo[1] = (int)round(2.4/m_res); cargo[2] = (int)round(3.0/m_res);
      roof_offset[0] = 4.0f; roof_offset[1] = 0.0f; roof_offset[2] = 2.0f;
      base_offset[0] = 0.0f; base_offset[1] = 0.0f; base_offset[2] = 0.7f;
      cargo_offset[0] = -1.3f; cargo_offset[1] = 0.0f; cargo_offset[2] = 2.9f;
    }

  // insert some measurements of free cells
  //Cargo: Truck's region above landing area
  for (int x=-cargo[0]; x<cargo[0]; x++) {
    for (int y=-cargo[1]; y<cargo[1]; y++) {
      for (int z=-cargo[2]; z<cargo[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value+cargo_offset[0], (float) y*step_value+cargo_offset[1], (float) z*step_value+cargo_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true);
      }
    }
  }


  // Truck's roof above drivers
  for (int x=-roof[0]; x<roof[0]; x++) {
    for (int y=-roof[1]; y<roof[1]; y++) {
      for (int z=-roof[2]; z<roof[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value+roof_offset[0], (float) y*step_value+roof_offset[1], (float) z*step_value+roof_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=-base[0]; x<base[0]; x++) {
    for (int y=-base[1]; y<base[1]; y++) {
      for (int z=-base[2]; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value+base_offset[0], (float) y*step_value+base_offset[1], (float) z*step_value+base_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  point3d end_pt(0, 0, 0);
  //m_octree->updateNode(end_pt, true);
  m_octree->prune();

  printf("Layers: %d %d\n", m_octree->tree_depth, (int)m_octree->tree_size);
}


void TruckOctomapServer::laneMarkerVisualization()
{
  ros::NodeHandle nh;
  visualization_msgs::Marker lane_list_marker;
  lane_list_marker.ns = "lanes";
  lane_list_marker.header.frame_id = std::string("/world");
  lane_list_marker.header.stamp = ros::Time().now();
  lane_list_marker.action = visualization_msgs::Marker::ADD;
  lane_list_marker.id = 0;
  lane_list_marker.type = visualization_msgs::Marker::LINE_LIST;

  lane_list_marker.pose.position.x = 0.0;
  lane_list_marker.pose.position.y = 0.0;
  lane_list_marker.pose.position.z = 0.0;

  lane_list_marker.pose.orientation.x = 0.0;
  lane_list_marker.pose.orientation.y = 0.0;
  lane_list_marker.pose.orientation.z = 0.0;
  lane_list_marker.pose.orientation.w = 1.0;
  lane_list_marker.scale.x = 0.2;
  lane_list_marker.scale.y = 0.2;
  lane_list_marker.scale.z = 0.2;
  lane_list_marker.color.a = 1.0;
  lane_list_marker.color.r = 1.0f;
  lane_list_marker.color.g = 1.0f;
  lane_list_marker.color.b = 1.0f;
  // Create the vertices for the points and lines
  for (uint32_t i = 0; i < 4; ++i)
    {
      float y = i * 3.5 - 5.25;

      geometry_msgs::Point p;
      p.x = 50.0;
      p.y = y;
      p.z = 0;

      // The line list needs two points for each line
      lane_list_marker.points.push_back(p);
      p.x = -20.0;
      lane_list_marker.points.push_back(p);
    }
  pub_lane_marker = nh.advertise<visualization_msgs::Marker>("lane_marker", 10);
  pub_lane_marker.publish(lane_list_marker);
}
