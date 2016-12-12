
#include <ros/ros.h>
#include <truck_server/TruckOctomapServer.h>
#include <unistd.h>

using namespace octomap_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_truck_server");
  try{
    TruckOctomapServer truck;
    truck.WriteTruckOctree(Pose6D(0,0,0,0,0,-M_PI/4));
    while(1)
      {
        //truck.publishTruckFullOctoMap(ros::Time().now());
        truck.publishTruckAll(ros::Time().now());
        usleep(1000000);
      }
    ros::spin();
  }
  catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
