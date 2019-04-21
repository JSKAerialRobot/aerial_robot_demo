// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef TRAJECTORY_PREDICTION_H
#define TRAJECTORY_PREDICTION_H

#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbzirc2020_task1_tasks/LinearKalmanFilter.h>

/* linear algebra */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

/* general header file */
#include <ros/ros.h>
#include <iostream>
#include <math.h>

using namespace linear_kalman_filter;
namespace trajectory_predictor{
  #define PI 3.141593
  #define MAP_NO_INFO 0
  #define MAP_POINT_INFO 1 // map info obtains from every single state
  #define MAP_REGION_INFO 2 // map info discretize into several regions
  struct mapPriorInfo{
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Eigen::Vector2d default_acc;
  };
  class TrajectoryPredictor{
  public:
    TrajectoryPredictor(ros::NodeHandle nh, ros::NodeHandle nhp);
    Eigen::VectorXd getPredictedState(double relative_time);
    Eigen::VectorXd getPredictedControlInput(double relative_time);
    bool checkPredictedResultsAvaiable(){return predicted_results_available_;}

  private:
    /* basic */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    nav_msgs::Odometry tracked_object_odom_;
    LinearKalmanFilter *lkf_;
    int state_dim_;
    int u_dim_;
    int z_dim_; // observation
    double filter_freq_;
    double predict_horizon_;

    bool filter_init_flag_;
    Eigen::VectorXd x_post_;
    Eigen::VectorXd cur_z_;
    bool predicted_results_available_;
    std::vector<Eigen::VectorXd> predicted_state_vec_;
    std::vector<Eigen::VectorXd> predicted_u_vec_;

    // map
    int map_mode_;
    double map_radius_;
    double map_cross_ang_;
    double object_default_height_;
    double object_default_vel_value_;
    double object_default_acc_value_;
    std::vector<mapPriorInfo> map_prior_info_vec_;

    ros::Subscriber sub_tracked_object_odom_;
    ros::Publisher pub_predicted_object_trajectory_;

    void updatePredictorFilter();
    void updateModel(Eigen::VectorXd cur_x, Eigen::MatrixXd &cur_F, Eigen::MatrixXd &cur_B, Eigen::MatrixXd &cur_H, Eigen::VectorXd &cur_u);
    void updateNoiseModel(Eigen::MatrixXd &Q, Eigen::MatrixXd &R);
    void trackedObjectOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void predictState();
    void visualizePredictedState();
    Eigen::VectorXd initStateFromObservation(Eigen::VectorXd &z);
    void updateMapInfo();
    Eigen::VectorXd getControlInputFromMap(Eigen::VectorXd &x);
  };
}

#endif
