#include <mbzirc2020_task1_tasks/GRANSAC.hpp>
#include <mbzirc2020_task1_tasks/Line2DModel.hpp>
#include <mbzirc2020_task1_tasks/Line3DModel.hpp>

#include <cmath>
#include <random>
#include <iostream>

#include <ros/ros.h>
#include<geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#include <tf/LinearMath/Transform.h>

class RansacLineFitting{
#define STOP_ESTIMATION 0
#define IN_ESTIMATION 1
public:
  RansacLineFitting(ros::NodeHandle nh, ros::NodeHandle nhp);
  void update();
  bool isEstimated();
  bool getNearestWaypoint(Eigen::Vector3d pos, Eigen::Vector3d &waypt);
  bool isNearTarget(Eigen::Vector3d pos);
  bool checkEstimationWithYawAng(double yaw);

private:
  /* basic */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber target_pt_sub_;
  ros::Publisher fitted_line_pub_;
  ros::Publisher fitted_points_pub_;

  int estimator_state_;
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> cand_points2d_;
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> cand_points3d_;
  int cand_points2d_init_size_;
  int cand_points3d_init_size_;
  int cand_points2d_max_size_;
  int cand_points3d_max_size_;
  GRANSAC::RANSAC<Line2DModel, 2> estimator_2d_;
  GRANSAC::RANSAC<Line3DModel, 2> estimator_3d_;
  double lpf_z_;
  double lpf_z_gain_;
  bool ransac_vis_flag_;
  bool ransac_3d_mode_;
  double target_pt_update_time_;
  double target_pt_dispear_time_thre_;
  double target_close_dist_thre_;
  double yaw_diff_thre_;

  void targetPointCallback(const geometry_msgs::PointStampedConstPtr & msg);
  void visualizeRansacLine();
  void visualizeRansacInliers();
  void initializeEstimatorParam();
};
