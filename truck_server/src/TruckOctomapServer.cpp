
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

TruckOctomapServer::~TruckOctomapServer() {
}

void TruckOctomapServer::init_param()
{
  //roof.assign((int)round(1.0/m_res), (int)round(1.5/m_res), (int)round(1.0/m_res));
  roof[0] = (int)round(1.0/m_res); roof[1] = (int)round(1.5/m_res); roof[2] = (int)round(1.0/m_res);
  //base.assign((int)round(2.5/m_res), (int)round(1.5/m_res), (int)round(1.5/m_res));
  base[0] = (int)round(2.5/m_res); base[1] = (int)round(1.5/m_res); base[2] = (int)round(1.0/m_res);
  //cargo.assign((int)round(1.5/m_res), (int)round(1.5/m_res), (int)round(1.0/m_res));
  cargo[0] = (int)round(1.5/m_res); cargo[1] = (int)round(1.5/m_res); cargo[2] = (int)round(0.5/m_res);

  step_value = (float)m_res / 2.0f;
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

void TruckOctomapServer::WriteTruckOctree(Pose6D rot_mat)
{
  printf("Resolution is %f\n", m_res);

  // Truck's roof above drivers
  for (int x=-roof[0]; x<roof[0]; x++) {
    for (int y=-roof[1]; y<roof[1]; y++) {
      for (int z=-roof[2]; z<roof[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value+0.75f, (float) y*step_value, (float) z*step_value+1.25f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=-base[0]; x<base[0]; x++) {
    for (int y=-base[1]; y<base[1]; y++) {
      for (int z=-base[2]; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value, (float) y*step_value, (float) z*step_value+0.5f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }


  // insert some measurements of free cells
  //Cargo: Truck's region above landing area
  for (int x=-cargo[0]; x<cargo[0]; x++) {
    for (int y=-cargo[1]; y<cargo[1]; y++) {
      for (int z=-cargo[2]; z<cargo[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value-0.5f, (float) y*step_value, (float) z*step_value+1.25f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

}

