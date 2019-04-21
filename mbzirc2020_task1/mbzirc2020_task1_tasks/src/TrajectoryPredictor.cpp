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

#include <mbzirc2020_task1_tasks/TrajectoryPredictor.h>
namespace trajectory_predictor{
  TrajectoryPredictor::TrajectoryPredictor(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;

    state_dim_ = 6; // p_x, v_x, p_y, v_y, p_z, v_z
    u_dim_ = 3; // a_x, a_y, a_z
    z_dim_ = 6; // p_x, v_x, p_y, v_y, p_z, v_z
    filter_freq_ = 100.0; // same with odom update frequency
    nhp_.param("predict_horizon", predict_horizon_, 2.0); // predict period of future state
    lkf_ = new LinearKalmanFilter(nh_, nhp_, state_dim_, u_dim_, z_dim_);
    x_post_ = Eigen::VectorXd::Zero(state_dim_);
    cur_z_ = Eigen::VectorXd::Zero(z_dim_);
    filter_init_flag_ = false;
    predicted_results_available_ = false;

    map_mode_ = MAP_POINT_INFO;
    if (map_mode_ == MAP_REGION_INFO)
      updateMapInfo();

    nhp_.param("radius", map_radius_, 16.0);
    nhp_.param("cross_angle", map_cross_ang_, PI / 4.0);
    nhp_.param("object_height", object_default_height_, 2.5);
    nhp_.param("linear_velocity", object_default_vel_value_, 5.0);
    object_default_acc_value_ = pow(object_default_vel_value_, 2.0) / map_radius_;
    updateMapInfo();

    std::string sub_tracked_object_odom_topic_name;
    nhp_.param("tracked_object_odom_topic_name", sub_tracked_object_odom_topic_name, std::string("/hawk/ground_truth/state"));
    sub_tracked_object_odom_ = nh_.subscribe<nav_msgs::Odometry>(sub_tracked_object_odom_topic_name, 1, &TrajectoryPredictor::trackedObjectOdomCallback, this);

    pub_predicted_object_trajectory_ = nh_.advertise<nav_msgs::Path>("/predicted_object_path", 1);
  }

  void TrajectoryPredictor::trackedObjectOdomCallback(const nav_msgs::OdometryConstPtr& msg){
    tracked_object_odom_ = *msg;

    updatePredictorFilter();
  }

  void TrajectoryPredictor::updatePredictorFilter(){
    cur_z_(0) = tracked_object_odom_.pose.pose.position.x;
    cur_z_(1) = tracked_object_odom_.twist.twist.linear.x;
    cur_z_(2) = tracked_object_odom_.pose.pose.position.y;
    cur_z_(3) = tracked_object_odom_.twist.twist.linear.y;
    cur_z_(4) = tracked_object_odom_.pose.pose.position.z;
    cur_z_(5) = tracked_object_odom_.twist.twist.linear.z;
    if (!filter_init_flag_){
      Eigen::VectorXd init_state = initStateFromObservation(cur_z_);
      Eigen::MatrixXd init_P_priori = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
      lkf_->initPrioriState(init_state, init_P_priori);
      x_post_ = init_state;
      filter_init_flag_ = true;
    }

    Eigen::MatrixXd cur_F = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    Eigen::MatrixXd cur_B = Eigen::MatrixXd::Zero(state_dim_, u_dim_);
    Eigen::MatrixXd cur_H = Eigen::MatrixXd::Zero(z_dim_, state_dim_);
    Eigen::VectorXd cur_u = Eigen::VectorXd::Zero(u_dim_);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(z_dim_, z_dim_);
    updateModel(x_post_, cur_F, cur_B, cur_H, cur_u);
    updateNoiseModel(Q, R);
    lkf_->setModel(cur_z_, cur_u, cur_F, cur_B, cur_H, Q, R);
    lkf_->update();
    x_post_ = lkf_->getPosterioriState();

    predictState();
    visualizePredictedState();
  }

  void TrajectoryPredictor::updateModel(Eigen::VectorXd cur_x, Eigen::MatrixXd &cur_F, Eigen::MatrixXd &cur_B, Eigen::MatrixXd &cur_H, Eigen::VectorXd &cur_u){
    for (int i = 0; i < state_dim_; ++i)
      cur_F(i, i) = 1.0;
    for (int i = 0; i < state_dim_ / 2; ++i)
      cur_F(2 * i, 2 * i + 1) = 1.0 / filter_freq_;

    for (int i = 0; i < state_dim_; ++i)
      cur_H(i, i) = 1.0;

    // task related
    for (int i = 0; i < state_dim_ / 2; ++i){
      cur_B(2 * i, i) = pow(1.0 / filter_freq_, 2) / 2.0;
      cur_B(2 * i + 1, i) = 1.0 / filter_freq_;
    }
    // cur_u is map-related (cur_x)
    cur_u = Eigen::VectorXd::Zero(u_dim_);

    if (map_mode_ == MAP_POINT_INFO){
      Eigen::Vector2d left_down_corner(-map_radius_ / tan(map_cross_ang_) * cos(map_cross_ang_),
                                       -map_radius_ / tan(map_cross_ang_) * sin(map_cross_ang_));
      Eigen::Vector2d right_down_corner(map_radius_ / tan(map_cross_ang_) * cos(map_cross_ang_),
                                        -map_radius_ / tan(map_cross_ang_) * sin(map_cross_ang_));
      Eigen::Vector2d left_circle_center(-map_radius_ / sin(map_cross_ang_), 0.0);
      Eigen::Vector2d right_circle_center(map_radius_ / sin(map_cross_ang_), 0.0);
      if (cur_x(0) <= left_down_corner(0)){ // enter left circle route
        double dist = sqrt(pow(cur_x(0) - left_circle_center(0), 2) +
                           pow(cur_x(2) - left_circle_center(1), 2));
        Eigen::Vector2d acc_normal(-(cur_x(0) - left_circle_center(0)) / dist,
                                   -(cur_x(2) - left_circle_center(1)) / dist);
        Eigen::Vector2d cur_u_xy = object_default_acc_value_ * acc_normal;
        cur_u << cur_u_xy(0), cur_u_xy(1), 0.0;
      }
      else if (cur_x(0) >= right_down_corner(0)){ // enter right circle route
        double dist = sqrt(pow(cur_x(0) - right_circle_center(0), 2) +
                           pow(cur_x(2) - right_circle_center(1), 2));
        Eigen::Vector2d acc_normal(-(cur_x(0) - right_circle_center(0)) / dist,
                                   -(cur_x(2) - right_circle_center(1)) / dist);
        Eigen::Vector2d cur_u_xy = object_default_acc_value_ * acc_normal;
        cur_u << cur_u_xy(0), cur_u_xy(1), 0.0;
      }
    }
    else if (map_mode_ == MAP_REGION_INFO)
      cur_u = getControlInputFromMap(cur_x);
  }

  void TrajectoryPredictor::updateNoiseModel(Eigen::MatrixXd &Q, Eigen::MatrixXd &R){
    Q = 0.01 * Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    R = 0.01 * Eigen::MatrixXd::Identity(z_dim_, z_dim_);
  }

  void TrajectoryPredictor::predictState(){
    Eigen::VectorXd x_priori = Eigen::VectorXd::Zero(state_dim_);
    Eigen::VectorXd x_post = Eigen::VectorXd::Zero(state_dim_);

    Eigen::MatrixXd cur_F = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    Eigen::MatrixXd cur_B = Eigen::MatrixXd::Zero(state_dim_, u_dim_);
    Eigen::MatrixXd cur_H = Eigen::MatrixXd::Zero(z_dim_, state_dim_);
    Eigen::VectorXd cur_u = Eigen::VectorXd::Zero(u_dim_);

    predicted_state_vec_.clear();
    predicted_u_vec_.clear();
    predicted_results_available_ = false;
    x_post = x_post_;
    for (int i = 0; i <= int(predict_horizon_ * filter_freq_); ++i){
      updateModel(x_post, cur_F, cur_B, cur_H, cur_u);
      x_priori = cur_F * x_post + cur_B * cur_u;
      predicted_state_vec_.push_back(x_priori);
      predicted_u_vec_.push_back(cur_u);
      x_post = x_priori; // assume residual term 0 in prediction
    }
    predicted_results_available_ = true;
  }

  Eigen::VectorXd TrajectoryPredictor::getPredictedState(double relative_time){
    int id = int(relative_time * filter_freq_);
    return predicted_state_vec_[id];
  }

  Eigen::VectorXd TrajectoryPredictor::getPredictedControlInput(double relative_time){
    int id = int(relative_time * filter_freq_);
    return predicted_u_vec_[id];
  }

  void TrajectoryPredictor::visualizePredictedState(){
    nav_msgs::Path predicted_path;
    predicted_path.header.frame_id = std::string("/world");
    predicted_path.header.stamp = ros::Time::now();
    predicted_path.header.seq = 0;
    for (int i = 0; i < predicted_state_vec_.size(); ++i){
      geometry_msgs::PoseStamped cur_pose;
      cur_pose.pose.position.x = predicted_state_vec_[i](0);
      cur_pose.pose.position.y = predicted_state_vec_[i](2);
      cur_pose.pose.position.z = predicted_state_vec_[i](4);
      predicted_path.poses.push_back(cur_pose);
    }
    pub_predicted_object_trajectory_.publish(predicted_path);
  }

  Eigen::VectorXd TrajectoryPredictor::initStateFromObservation(Eigen::VectorXd &z){
    Eigen::VectorXd x(state_dim_);
    for (int i = 0; i < state_dim_; ++i)
      x(i) = z(i);
    return x;
  }

  void TrajectoryPredictor::updateMapInfo(){
    Eigen::Vector2d left_down_corner(-map_radius_ / tan(map_cross_ang_) * cos(map_cross_ang_),
                                     -map_radius_ / tan(map_cross_ang_) * sin(map_cross_ang_));
    Eigen::Vector2d right_down_corner(map_radius_ / tan(map_cross_ang_) * cos(map_cross_ang_),
                                      -map_radius_ / tan(map_cross_ang_) * sin(map_cross_ang_));
    Eigen::Vector2d right_up_corner = -left_down_corner;
    Eigen::Vector2d left_up_corner = -right_down_corner;
    Eigen::Vector2d left_circle_center(-map_radius_ / sin(map_cross_ang_), 0.0);
    Eigen::Vector2d right_circle_center(map_radius_ / sin(map_cross_ang_), 0.0);

    // todo: change to ros param
    double circle_route_angle_gap = PI / 4.0;
    int circle_route_angle_seg = 6;

    mapPriorInfo first_straight;
    first_straight.start = left_down_corner;
    first_straight.end = right_up_corner;
    first_straight.default_acc = Eigen::Vector2d::Zero();
    map_prior_info_vec_.push_back(first_straight);
    for (int i = 0; i < circle_route_angle_seg; ++i){
      mapPriorInfo circle_mark_point;
      double point_start_ang = (PI - map_cross_ang_) - circle_route_angle_gap * i;
      double point_mid_ang = point_start_ang - circle_route_angle_gap / 2.0;
      double point_end_ang = point_start_ang - circle_route_angle_gap;
      circle_mark_point.start = right_circle_center + map_radius_ * Eigen::Vector2d(cos(point_start_ang), sin(point_start_ang));
      circle_mark_point.end = right_circle_center + map_radius_ * Eigen::Vector2d(cos(point_end_ang), sin(point_end_ang));
      circle_mark_point.default_acc = object_default_acc_value_ * Eigen::Vector2d(-cos(point_mid_ang), -sin(point_mid_ang));
      map_prior_info_vec_.push_back(circle_mark_point);
    }

    mapPriorInfo second_straight;
    second_straight.start = right_down_corner;
    second_straight.end = left_up_corner;
    second_straight.default_acc = Eigen::Vector2d::Zero();
    map_prior_info_vec_.push_back(second_straight);
    for (int i = 0; i < circle_route_angle_seg; ++i){
      mapPriorInfo circle_mark_point;
      double point_start_ang = (PI - map_cross_ang_) + circle_route_angle_gap * i;
      double point_mid_ang = point_start_ang + circle_route_angle_gap / 2.0;
      double point_end_ang = point_start_ang + circle_route_angle_gap;
      circle_mark_point.start = right_circle_center + map_radius_ * Eigen::Vector2d(cos(point_start_ang), sin(point_start_ang));
      circle_mark_point.end = right_circle_center + map_radius_ * Eigen::Vector2d(cos(point_end_ang), sin(point_end_ang));
      circle_mark_point.default_acc = object_default_acc_value_ * Eigen::Vector2d(-cos(point_mid_ang), -sin(point_mid_ang));
      map_prior_info_vec_.push_back(circle_mark_point);
    }
  }

  Eigen::VectorXd TrajectoryPredictor::getControlInputFromMap(Eigen::VectorXd &x){
    // todo
    Eigen::VectorXd u = Eigen::VectorXd::Zero(u_dim_);
    return u;
  }
}
