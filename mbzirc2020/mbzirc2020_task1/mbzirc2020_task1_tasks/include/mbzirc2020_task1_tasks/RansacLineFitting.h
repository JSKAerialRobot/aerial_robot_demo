#include <mbzirc2020_task1_tasks/GRANSAC.hpp>
#include <mbzirc2020_task1_tasks/LineModel.hpp>

#include <cmath>
#include <random>
#include <iostream>

#include <ros/ros.h>
#include<geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class RansacLineFitting{
public:
  RansacLineFitting(ros::NodeHandle nh, ros::NodeHandle nhp);
  void update();
  void update_ori();

private:
  /* basic */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber target_pt_sub_;
  ros::Publisher fitted_line_pub_;
  ros::Publisher fitted_points_pub_;

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> cand_points_;
  int cand_points_max_size_;
  double lpf_z_;
  double lpf_z_gain_;
  bool ransac_vis_flag_;
  GRANSAC::RANSAC<Line2DModel, 2> estimator_2d_;

  void targetPointCallback(const geometry_msgs::PointStampedConstPtr & msg);
  void visualizeRansacLine();
  void visualizeRansacInliers();
};
