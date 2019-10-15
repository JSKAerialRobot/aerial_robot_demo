#include <mbzirc2020_task1_tasks/TrajectoryTracker.h>

using namespace trajectory_tracker;
int main (int argc, char **argv)
{
  ros::init (argc, argv, "trajectory_tracker_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TrajectoryTracker trajectory_tracker(nh, nh_private);
  ros::spin();
  return 0;
}
