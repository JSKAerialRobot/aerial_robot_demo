#include <mbzirc2020_task1_tasks/TrajectoryPredictor.h>

using namespace trajectory_predictor;
int main (int argc, char **argv)
{
  ros::init (argc, argv, "trajectory_predictor_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TrajectoryPredictor trajectory_predictor(nh, nh_private);
  ros::spin();
  return 0;
}
