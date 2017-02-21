
#include <truck_server/TruckOctomapServer.h>
#include <string>

using namespace octomap;
using namespace octomath;
using namespace octomap_server;

TruckOctomapServer::TruckOctomapServer(double resolution, int tree_depth) :
  OctomapServer(resolution, tree_depth)
{

  ros::NodeHandle private_nh("~");
  // Shi
  //private_nh.param("resolution", m_res, 0.1);
  //private_nh.param("max_tree_depth", m_max_tree_depth, 10);
  private_nh.param("frame_id", m_worldFrameId, (std::string)"world");
  private_nh.param("route_name", m_route_name, (std::string)"track");
  private_nh.param("route_radius", m_route_radius, 20.0f);

  init_param();
}

TruckOctomapServer::~TruckOctomapServer() {}

void TruckOctomapServer::init_param()
{
  // Shi
  //m_octree->setResolution(m_res);

  m_step_value = (float)m_res / 2.0f;

  m_octree->enableChangeDetection(true);

  m_pub_lane_marker = this->m_nh.advertise<visualization_msgs::Marker>("lane_marker", 10);

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
  float roof_size[3], base_size[3], cargo_size[3];
  // For uav safety margin
  float uav_safety_margin_size[3] = {0.5f, 0.5f, 0.2f}, uav_safety_margin[3];

  // truck for challenge
  if (type == 0)
    {
      base_size[0] = 3.5f; base_size[1] = 1.5f; base_size[2] = 0.7f;
      roof_size[0] = 1.5f; roof_size[1] = 1.5f; roof_size[2] = 1.0f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }
  // truck with out roof
  else if (type == -1)
    {
      base_size[0] = 3.6f; base_size[1] = 1.6f; base_size[2] = 0.7f;
      roof_size[0] = 0.0f; roof_size[1] = 0.0f; roof_size[2] = 0.0f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }
  // sedan vehicle
  else if (type == 1)
    {
      base_size[0] = 4.8f; base_size[1] = 2.0f; base_size[2] = 0.8f;
      roof_size[0] = 3.0f; roof_size[1] = 2.0f; roof_size[2] = 0.9f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }
  // big truck
  else if (type == 2)
    {
      base_size[0] = 6.0f; base_size[1] = 2.4f; base_size[2] = 3.4f;
      roof_size[0] = 0.0f; roof_size[1] = 0.0f; roof_size[2] = 0.0f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }

  // For uav safety margin
  for (int i = 0; i < 3; ++i){
    base_size[i] += 2*uav_safety_margin_size[i];
    // for vehicles not having roof
    if (type != -1 and type != 2){
      roof_size[i] += 2*uav_safety_margin_size[i];
    }
    if (cargo_size[0] > 0.1)
      cargo_size[i] += 2*uav_safety_margin_size[i];
  }

  for (int i = 0; i < 3; ++i){
    base[i] = (int)round(base_size[i]/m_res);
    roof[i] = (int)round(roof_size[i]/m_res);
    cargo[i] = (int)round(cargo_size[i]/m_res);
  }
  base_offset[0] = -base_size[0]/2.0f; base_offset[1] = -base_size[1]/2.0f; base_offset[2] = 0.0f;
  // sedan's roof is in the middle
  if (type == 1)
    roof_offset[0] = -roof_size[0]/2.0f;
  else
    roof_offset[0] = base_size[0]/2.0f - roof_size[0];
  roof_offset[1] = -roof_size[1]/2.0f; roof_offset[2] = base_size[2];
  // No cargo
  if (cargo_size[0] < 0.1){
    cargo_offset[0] = 0.0f; cargo_offset[1] = 0.0f; cargo_offset[2] = 0.0f;
  }
  else{
    cargo_offset[0] = -base_size[0]/2.0f; cargo_offset[1] = -cargo_size[1]/2.0f; cargo_offset[2] = base_size[2];
  }


  // Judge whether start point is on boarder, in case octo map give seperate thin plannar.
  for (int i = 0; i < 3; ++i)
    {
      int res_100 = int(m_res*100);
      if (int(roof_offset[i]*100) % res_100 == 0)
        roof_offset[i] += m_res/2.0f;
      if (int(base_offset[i]*100) % res_100 == 0)
        base_offset[i] += m_res/2.0f;
      if (int(cargo_offset[i]*100) % res_100 == 0)
        cargo_offset[i] += m_res/2.0f;
    }

  // insert some measurements of free cells
  //Cargo: Truck's region above landing area
  for (int x=0; x<cargo[0]; x++) {
    for (int y=0; y<cargo[1]; y++) {
      for (int z=0; z<cargo[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+cargo_offset[0], (float) y*m_res+cargo_offset[1], (float) z*m_res+cargo_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true);
      }
    }
  }


  // Truck's roof above drivers
  for (int x=0; x<roof[0]; x++) {
    for (int y=0; y<roof[1]; y++) {
      for (int z=0; z<roof[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+roof_offset[0], (float) y*m_res+roof_offset[1], (float) z*m_res+roof_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=0; x<base[0]; x++) {
    for (int y=0; y<base[1]; y++) {
      for (int z=0; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+base_offset[0], (float) y*m_res+base_offset[1], (float) z*m_res+base_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }


  m_octree->prune();
  m_octree->prune();

  //printf("Layers: %d %d\n", m_octree->tree_depth, (int)m_octree->tree_size);
}


void TruckOctomapServer::WriteUavSafeBorderOctree(int type, Pose6D rot_mat)
{
  int roof[3], base[3], cargo[3];
  int roof_origin[3], base_origin[3], cargo_origin[3];
  float roof_offset[3], base_offset[3], cargo_offset[3];
  float roof_size[3], base_size[3], cargo_size[3];
  // For uav safety margin
  float uav_safety_margin_size[3] = {2*0.6f, 2*0.6f, 0.2f}, uav_safety_margin[3];

  // truck for challenge
  if (type == 0)
    {
      base_size[0] = 3.5f; base_size[1] = 1.5f; base_size[2] = 0.7f;
      roof_size[0] = 1.5f; roof_size[1] = 1.5f; roof_size[2] = 1.0f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }
  // sedan vehicle
  else if (type == 1)
    {
      base_size[0] = 4.8f; base_size[1] = 2.0f; base_size[2] = 0.8f;
      roof_size[0] = 2.4f; roof_size[1] = 2.0f; roof_size[2] = 0.9f;
      cargo_size[0] = 0.0f; cargo_size[1] = 0.0f; cargo_size[2] = 0.0f;
    }
  // big truck
  else if (type == 2)
    {
      base_size[0] = 10.0f; base_size[1] = 2.4f; base_size[2] = 1.4f;
      roof_size[0] = 2.0f; roof_size[1] = 2.4f; roof_size[2] = 1.2f;
      cargo_size[0] = 7.4f; cargo_size[1] = 2.4f; cargo_size[2] = 3.0f;
    }

  // For uav safety margin
  for (int i = 0; i < 3; ++i){
    base_origin[i] = (int)round(base_size[i]/m_res);
    roof_origin[i] = (int)round(roof_size[i]/m_res);
    cargo_origin[i] = (int)round(cargo_size[i]/m_res);
    base_size[i] += uav_safety_margin_size[i];
    roof_size[i] += uav_safety_margin_size[i];
    if (cargo_size[0] > 0.1)
      cargo_size[i] += uav_safety_margin_size[i];
  }

  for (int i = 0; i < 3; ++i){
    base[i] = (int)round(base_size[i]/m_res);
    roof[i] = (int)round(roof_size[i]/m_res);
    cargo[i] = (int)round(cargo_size[i]/m_res);
  }
  base_offset[0] = -base_size[0]/2.0f; base_offset[1] = -base_size[1]/2.0f; base_offset[2] = 0.0f;
  // sedan's roof is in the middle
  if (type == 1)
    roof_offset[0] = -roof_size[0]/2.0f;
  else
    roof_offset[0] = base_size[0]/2.0f - roof_size[0];
  roof_offset[1] = -roof_size[1]/2.0f; roof_offset[2] = base_size[2];
  // No cargo
  if (cargo_size[0] < 0.1){
    cargo_offset[0] = 0.0f; cargo_offset[1] = 0.0f; cargo_offset[2] = 0.0f;
  }
  else{
    cargo_offset[0] = -base_size[0]/2.0f; cargo_offset[1] = -cargo_size[1]/2.0f; cargo_offset[2] = base_size[2];
  }

  // Judge whether start point is on boarder, in case octo map give seperate thin plannar.
  for (int i = 0; i < 3; ++i)
    {
      int res_100 = int(m_res*100);
      if (int(roof_offset[i]*100) % res_100 == 0)
        roof_offset[i] += m_res/2.0f;
      if (int(base_offset[i]*100) % res_100 == 0)
        base_offset[i] += m_res/2.0f;
      if (int(cargo_offset[i]*100) % res_100 == 0)
        cargo_offset[i] += m_res/2.0f;
    }

  // insert some measurements of free cells
  //Cargo: Truck's region above landing area
  for (int x=0; x<cargo[0]; x++) {
    for (int y=0; y<cargo[1]; y++) {
      for (int z=0; z<cargo[2]; z++) {
        if (x <= (cargo[0]+cargo_origin[0])/2 && x >= (cargo[0]-cargo_origin[0])/2 && y <= (cargo[1]+cargo_origin[1])/2 && y >= (cargo[1]-cargo_origin[1])/2 && z <= cargo_origin[2])
          continue;
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+cargo_offset[0], (float) y*m_res+cargo_offset[1], (float) z*m_res+cargo_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true);
      }
    }
  }


  // Truck's roof above drivers
  // truck without roof
  for (int x=0; x<roof[0]; x++) {
    for (int y=0; y<roof[1]; y++) {
      for (int z=0; z<roof[2]; z++) {
        if (x <= (roof[0]+roof_origin[0])/2 && x >= (roof[0]-roof_origin[0])/2 && y <= (roof[1]+roof_origin[1])/2 && y >= (roof[1]-roof_origin[1])/2 && z <= roof_origin[2])
          continue;
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+roof_offset[0], (float) y*m_res+roof_offset[1], (float) z*m_res+roof_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=0; x<base[0]; x++) {
    for (int y=0; y<base[1]; y++) {
      for (int z=0; z<base[2]; z++) {
        if (x <= (base[0]+base_origin[0])/2 && x >= (base[0]-base_origin[0])/2 && y <= (base[1]+base_origin[1])/2 && y >= (base[1]-base_origin[1])/2 && z <= base_origin[2])
          continue;
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+base_offset[0], (float) y*m_res+base_offset[1], (float) z*m_res+base_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }


  m_octree->prune();
  m_octree->prune();

  printf("Layers: %d %d\n", m_octree->tree_depth, (int)m_octree->tree_size);
}


void TruckOctomapServer::WriteObstacleOctree(int type, Pose6D rot_mat)
{
  int roof[3], base[3];
  float roof_offset[3], base_offset1[3], base_offset2[3];
  float roof_size[3], base_size[3];
  // For uav safety margin
  float uav_safety_margin_size[3] = {0.5f, 0.5f, 0.2f}, uav_safety_margin[3];

  // bridge
  if (type == 0)
    {
      base_size[0] = 5.0f; base_size[1] = 2.0f; base_size[2] = 7.0f;
      roof_size[0] = 5.0f; roof_size[1] = 26.0f; roof_size[2] = 2.0f;
    }

  // For uav safety margin
  for (int i = 0; i < 3; ++i){
    base_size[i] += 2*uav_safety_margin_size[i];
    roof_size[i] += 2*uav_safety_margin_size[i];
  }

  for (int i = 0; i < 3; ++i){
    base[i] = (int)round(base_size[i]/m_res);
    roof[i] = (int)round(roof_size[i]/m_res);
  }
  roof_offset[0] = 0.0f; roof_offset[1] = -roof_size[1]/2.0f; roof_offset[2] = base_size[2];
  base_offset1[0] = 0.0f; base_offset1[1] = roof_size[1]/2.0f-base_size[1]; base_offset1[2] = 0.0f;
  base_offset2[0] = 0.0f; base_offset2[1] = -roof_size[1]/2.0f; base_offset2[2] = 0.0f;

  // Judge whether start point is on boarder, in case octo map give seperate thin plannar.
  for (int i = 0; i < 3; ++i)
    {
      int res_100 = int(m_res*100);
      if (int(roof_offset[i]*100) % res_100 == 0)
        roof_offset[i] += m_res/2.0f;
      if (int(base_offset1[i]*100) % res_100 == 0)
        base_offset1[i] += m_res/2.0f;
      if (int(base_offset2[i]*100) % res_100 == 0)
        base_offset2[i] += m_res/2.0f;
    }

  // roof above drivers
  for (int x=0; x<roof[0]; x++) {
    // For bridge, in order to extend its double side to longer, otherwise just for (int y=0; y<roof[1]; y++) 
    for (int y=-roof[1]; y<2*roof[1]; y++) {
      for (int z=0; z<roof[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+roof_offset[0], (float) y*m_res+roof_offset[1], (float) z*m_res+roof_offset[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=0; x<base[0]; x++) {
    for (int y=0; y<base[1]; y++) {
      for (int z=0; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+base_offset1[0], (float) y*m_res+base_offset1[1], (float) z*m_res+base_offset1[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  for (int x=0; x<base[0]; x++) {
    for (int y=0; y<base[1]; y++) {
      for (int z=0; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*m_res+base_offset2[0], (float) y*m_res+base_offset2[1], (float) z*m_res+base_offset2[2]));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }


  m_octree->prune();
  m_octree->prune();

  printf("Layers: %d %d\n", m_octree->tree_depth, (int)m_octree->tree_size);
  ROS_INFO("Obstacle octree is generated.");
}


void TruckOctomapServer::laneMarkerVisualization()
{
  visualization_msgs::Marker lane_strip_marker;
  lane_strip_marker.ns = "lanes";
  lane_strip_marker.header.frame_id = std::string("/world");
  lane_strip_marker.header.stamp = ros::Time().now();
  lane_strip_marker.action = visualization_msgs::Marker::ADD;
  lane_strip_marker.id = 0;
  lane_strip_marker.type = visualization_msgs::Marker::LINE_STRIP;

  lane_strip_marker.pose.position.x = 0.0;
  lane_strip_marker.pose.position.y = 0.0;
  lane_strip_marker.pose.position.z = 0.0;

  lane_strip_marker.pose.orientation.x = 0.0;
  lane_strip_marker.pose.orientation.y = 0.0;
  lane_strip_marker.pose.orientation.z = 0.0;
  lane_strip_marker.pose.orientation.w = 1.0;
  lane_strip_marker.scale.x = 0.2;
  lane_strip_marker.scale.y = 0.2;
  lane_strip_marker.scale.z = 0.2;
  lane_strip_marker.color.a = 1.0;
  lane_strip_marker.color.r = 1.0f;
  lane_strip_marker.color.g = 1.0f;
  lane_strip_marker.color.b = 1.0f;
  for (uint32_t i = 0; i < 4; ++i){
    // Create the vertices for the points and lines
    //double radius = m_route_radius + i * 3.5 - 5.25;
    double radius = m_route_radius + i * 5 - 7.5;
    if (m_route_name == (std::string)"circle"){
      for (int j = 0; j <= 360; ++j){
        geometry_msgs::Point p;
        p.x = radius * sin(j/180.0*3.14);
        p.y = -radius * cos(j/180.0*3.14);
        p.z = 0;

        // The line list needs two points for each line
        lane_strip_marker.points.push_back(p);
      }
    }
  }
  m_pub_lane_marker.publish(lane_strip_marker);
}
