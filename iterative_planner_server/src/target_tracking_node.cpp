#include <iterative_planner_server/IterativePlanner.h>
using namespace octomap_server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_planner_server");

  IterativePlanner track_planner;
  track_planner.onInit();

  ros::spin();
 return 0;
}
