#include <truck_server/octomap_truck_server_node.h>
using namespace octomap_server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_truck_server");

  TruckServerNode truck_node;
  truck_node.onInit();

  ros::spin();
 return 0;
}
