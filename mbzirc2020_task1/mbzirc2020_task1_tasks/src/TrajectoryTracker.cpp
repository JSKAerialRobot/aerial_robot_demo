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

#include <mbzirc2020_task1_tasks/TrajectoryTracker.h>
namespace trajectory_tracker{
  TrajectoryTracker::TrajectoryTracker(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;

    nhp_.param("replan_timer_period", replan_timer_period_, 0.1);
    nhp_.param("predict_horizon", kf_predict_horizon_, 4.0);
    nhp_.param("tracking_offset_z", tracking_offset_z_, 1.0);
    nhp_.param("primitive_candidates_num", primitive_candidates_num_, 6);
    nhp_.param("primitive_period_step", primitive_period_step_, 0.3);
    nhp_.param("primitive_period_base", primitive_period_base_, 0.3);
    nhp_.param("primitive_visualize_flag", primitive_visualize_flag_, false);
    nhp_.param("treasure_box_x", treasure_box_pos_(0), 0.0);
    nhp_.param("treasure_box_y", treasure_box_pos_(1), 0.0);
    treasure_box_pos_(2) = 1.0;

    primitive_ = new MotionSinglePrimitive();
    replan_prev_time_ = 0.0;
    tracking_state_ = PRE_TRACKING;
    immediate_replan_flag_ = false;

    pub_tracking_trajectory_ = nh_.advertise<nav_msgs::Path>("/track/vis/planned_path", 1);
    pub_grabbing_primitive_trajectory_ = nh_.advertise<nav_msgs::Path>("/track/vis/grab_primitive", 1);
    pub_tracking_target_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/track/vis/target_marker", 1);
    pub_tracking_primitive_params_ = nh_.advertise<mbzirc2020_task1_tasks::PrimitiveParams>("/track/primitive_params", 1);

    sub_host_robot_odom_ = nh_.subscribe<nav_msgs::Odometry>("/uav/cog/odom", 1, &TrajectoryTracker::hostRobotOdomCallback, this);
    sub_host_robot_imu_ = nh_.subscribe<sensor_msgs::Imu>("/ros_imu", 1, &TrajectoryTracker::hostRobotImuCallback, this);
    sub_host_robot_task_command_ = nh_.subscribe<std_msgs::UInt8>("/track/task/command", 1, &TrajectoryTracker::hostRobotTaskCommandCallback, this);

    predictor_thread_ = boost::thread(boost::bind(&TrajectoryTracker::predictorThread, this));
    replan_timer_ = nh_.createTimer(ros::Duration(0.001), &TrajectoryTracker::replanCallback, this);
    // todo: think about whether thread is needed for predictor
    object_trajectory_predictor_ = new TrajectoryPredictor(nh_, nhp_);
  }

  void TrajectoryTracker::predictorThread(){
    // boost::lock_guard<boost::mutex> lock(mutex_);
    // object_trajectory_predictor_ = new TrajectoryPredictor(nh_, nhp_);
  }

  void TrajectoryTracker::replanCallback(const ros::TimerEvent& event){
    // boost::lock_guard<boost::mutex> lock(mutex_);
    if (!object_trajectory_predictor_->checkPredictedResultsAvaiable())
      return;

    double cur_time = ros::Time::now().toSec();
    if (cur_time - replan_prev_time_ > replan_timer_period_ || immediate_replan_flag_){
      replanImpl();
      immediate_replan_flag_ = false;
      replan_prev_time_ = cur_time;
    }
  }

  void TrajectoryTracker::replanImpl(){
    Eigen::VectorXd cur_state = object_trajectory_predictor_->getPredictedState(0.0); // p_x, v_x, a_x, p_y, v_y, a_y, p_z, v_z, a_z

    double period = 1.0;
    double dist_xy = sqrt(pow(cur_state(0) - host_robot_odom_.pose.pose.position.x, 2) +
                          pow(cur_state(3) - host_robot_odom_.pose.pose.position.y, 2));
    double max_chase_delta_speed = 1.0;
    if (period < dist_xy / max_chase_delta_speed){
      period = dist_xy / max_chase_delta_speed;
      if (period > kf_predict_horizon_)
        period = kf_predict_horizon_;
    }
    if (tracking_state_ == PRE_TRACKING){
      tracking_state_ = KEEP_TRACKING;
      primitive_period_ = period;
    }
    else if (tracking_state_ == KEEP_TRACKING)
      primitive_period_ = period;
    else if (tracking_state_ == START_GRAPPING){
      tracking_state_ = IN_GRAPPING;
      // replan_timer_period_ = 0.1; // Not working
      /* find the primitive with minimum snap energy */
      double min_energy = -1;
      for (int i = 0; i < primitive_candidates_num_; ++i){
        double period_candidate = primitive_period_base_ + primitive_period_step_ * i;
        generatePrimitive(period_candidate);
        double cur_energy = primitive_->getPrimitiveEnergy();
        if (min_energy < 0 || cur_energy < min_energy){
          min_energy = cur_energy;
          period = period_candidate;
        }
        if (primitive_visualize_flag_)
          visualizationPrimitive(PRIMITIVE_MODE);
      }
      primitive_period_ = period;
    }
    else if (tracking_state_ == IN_GRAPPING){
      primitive_period_ -= ros::Time::now().toSec() - replan_prev_time_;
      if (primitive_period_ < 0.0){
        tracking_state_ = KEEP_STILL;
        primitive_period_ = 2.0;
      }
      /* during grapping, when period is too small, no need to replan */
      else if (primitive_period_ < 0.3)
        return;
    }
    else if (tracking_state_ == KEEP_STILL){
      return;
    }
    else if (tracking_state_ == QUIT_TASK){
      tracking_state_ = KEEP_STILL;
      primitive_period_ = 2.0;
    }
    else if (tracking_state_ == RELEASE_OBJECT){
      primitive_period_ = 1.0;
    }
    else if (tracking_state_ == CRUISE_TREASURE_BOX){
      double dist_treasure_box = sqrt(pow(treasure_box_pos_(0) - host_robot_odom_.pose.pose.position.x, 2) +
                                      pow(treasure_box_pos_(1) - host_robot_odom_.pose.pose.position.y, 2));
      primitive_period_ = dist_treasure_box / 2.0;

    }
    replan_timer_period_ = primitive_period_ - 0.2;
    if (replan_timer_period_ < 0.0){
      ROS_ERROR("[TrajectoryTracker] replan_timer_period is less than 0");
      replan_timer_period_ = 0.1;
    }

    generatePrimitive(primitive_period_);
    publishPrimitiveParam();
    visualizationPrimitive();
  }

  void TrajectoryTracker::generatePrimitive(double period){
    Eigen::VectorXd cur_state_full = Eigen::VectorXd::Zero(3 * 3);
    Eigen::VectorXd end_state_full = Eigen::VectorXd::Zero(3 * 3);
    cur_state_full << host_robot_odom_.pose.pose.position.x, host_robot_odom_.twist.twist.linear.x, host_robot_acc_world_(0), // x axis
      host_robot_odom_.pose.pose.position.y, host_robot_odom_.twist.twist.linear.y, host_robot_acc_world_(1), // y axis
      host_robot_odom_.pose.pose.position.z, host_robot_odom_.twist.twist.linear.z, host_robot_acc_world_(2); // z axis

    if (tracking_state_ == KEEP_TRACKING){
      end_state_full = object_trajectory_predictor_->getPredictedState(period);
      end_state_full(6) -= tracking_offset_z_; // add offset in z axis // todo
    }
    else if (tracking_state_ == KEEP_STILL || tracking_state_ == RELEASE_OBJECT){
      end_state_full(0) = cur_state_full(0) + host_robot_odom_.twist.twist.linear.x * period / 2.0; // to decelerate smoothly
      end_state_full(3) = cur_state_full(3) + host_robot_odom_.twist.twist.linear.y * period / 2.0;
      end_state_full(6) = cur_state_full(6); // add offset in z axis // todo
    }
    else if (tracking_state_ == CRUISE_TREASURE_BOX){
      for (int i = 0; i < 3; ++i)
        for (int j = 1; j < 3; ++j)
          cur_state_full(i * 3 + j) = 0.0; // set start vel/acc 0
      end_state_full = Eigen::VectorXd::Zero(3 * 3);
      for (int i = 0; i < 3; ++i)
        end_state_full(3 * i) = treasure_box_pos_(i);
      tracking_state_ = KEEP_STILL; // quit CRUISE_TREASURE_BOX state
    }
    else // IN_GRAPPING
      end_state_full = object_trajectory_predictor_->getPredictedState(period);
    convertToMPState(x_start_, cur_state_full);
    convertToMPState(x_end_, end_state_full);

    primitive_->init(period, x_start_, x_end_);
  }

  void TrajectoryTracker::publishPrimitiveParam(){
    mbzirc2020_task1_tasks::PrimitiveParams param_msg;
    std::vector<Eigen::VectorXd> primtive_param_vec = primitive_->getPrimitiveParams();
    int primitive_order = primitive_->getPolynomialOrder();
    int primitive_dim = primitive_->getStateDim();
    if (primitive_dim != 3){
      ROS_ERROR("primitive dimension is not 3, the x, y, z params could not be pulished.");
      return;
    }
    param_msg.order = primitive_order;
    param_msg.header.frame_id = "/world";
    param_msg.header.stamp = ros::Time::now(); // todo: real trajectory start time
    param_msg.period = primitive_->getPeriod();
    param_msg.x_params.resize(primitive_order);
    for (int i = 0; i < primitive_order; ++i)
      param_msg.x_params[i] = primtive_param_vec[0](i);
    param_msg.y_params.resize(primitive_order);
    for (int i = 0; i < primitive_order; ++i)
      param_msg.y_params[i] = primtive_param_vec[1](i);
    param_msg.z_params.resize(primitive_order);
    for (int i = 0; i < primitive_order; ++i)
      param_msg.z_params[i] = primtive_param_vec[2](i);

    if (tracking_state_ == IN_GRAPPING)
      param_msg.grap_flag = true;
    else if (tracking_state_ == RELEASE_OBJECT){
      param_msg.grap_flag = true;
      tracking_state_ = KEEP_STILL; // quit RELEASE_OBJECT state
    }
    else
      param_msg.grap_flag = false;

    param_msg.psi_params.resize(primitive_order);
    // todo: since euler intepolation has problem of jumping, directly give the target psi
    // tf::Quaternion q(host_robot_imu_.orientation.x,
    //                  host_robot_imu_.orientation.y,
    //                  host_robot_imu_.orientation.z,
    //                  host_robot_imu_.orientation.w);
    // tf::Matrix3x3  uav_rot_mat(q);
    // tfScalar uav_roll, uav_pitch, uav_psi;
    // uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_psi);
    // param_msg.psi_params[0] = uav_psi;

    if (tracking_state_ == KEEP_TRACKING || tracking_state_ == IN_GRAPPING){
      Eigen::VectorXd end_state_full = object_trajectory_predictor_->getPredictedState(param_msg.period);
      double target_psi = atan2(end_state_full(4), end_state_full(1));
      double psi_offset_uav = PI + 0.785;
      param_msg.psi_params[0] = target_psi + psi_offset_uav;
      while (param_msg.psi_params[0] > PI)
        param_msg.psi_params[0] -= 2.0 * PI;
    }
    pub_tracking_primitive_params_.publish(param_msg);
  }

  void TrajectoryTracker::visualizationPrimitive(int mode){
    nav_msgs::Path tracking_path;
    tracking_path.header.frame_id = std::string("/world");
    tracking_path.header.stamp = ros::Time::now();
    tracking_path.header.seq = 1;
    double period = primitive_->getPeriod();
    double traj_vis_time_resolution = 0.01;
    for (int i = 0; i < int(period / traj_vis_time_resolution); ++i){
      MPState waypoint = primitive_->getWaypointState(i * traj_vis_time_resolution);
      geometry_msgs::PoseStamped waypoint_pose;
      waypoint_pose.pose.position.x = waypoint.state[0](0);
      waypoint_pose.pose.position.y = waypoint.state[1](0);
      waypoint_pose.pose.position.z = waypoint.state[2](0);
      tracking_path.poses.push_back(waypoint_pose);
    }
    if (mode == SELECTED_TRAJECTORY_MODE)
      pub_tracking_trajectory_.publish(tracking_path);
    else
      pub_grabbing_primitive_trajectory_.publish(tracking_path);

    visualization_msgs::MarkerArray target_marker_array;
    visualization_msgs::Marker target_marker;
    target_marker.header = tracking_path.header;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.id = 0;
    target_marker.pose.position.x = x_start_.state[0](0);
    target_marker.pose.position.y = x_start_.state[1](0);
    target_marker.pose.position.z = x_start_.state[2](0);
    target_marker.scale.x = 0.5;
    target_marker.scale.y = 0.5;
    target_marker.scale.z = 0.5;
    target_marker.color.a = 1.0;
    target_marker.color.r = 0.0f;
    target_marker.color.g = 1.0f;
    target_marker.color.b = 0.0f;
    target_marker_array.markers.push_back(target_marker);
    // end target
    target_marker.id = 1;
    target_marker.pose.position.x = x_end_.state[0](0);
    target_marker.pose.position.y = x_end_.state[1](0);
    target_marker.pose.position.z = x_end_.state[2](0);
    target_marker.color.r = 1.0f;
    target_marker.color.g = 0.0f;
    target_marker.color.b = 0.0f;
    target_marker_array.markers.push_back(target_marker);
    pub_tracking_target_markers_.publish(target_marker_array);
  }

  void TrajectoryTracker::convertToMPState(MPState &x, Eigen::VectorXd state){
    int n_axis = state.size() / 3;
    x.state.resize(n_axis);
    for (int i = 0; i < n_axis; ++i)
      for (int j = 0; j < 3; ++j) // pos, vel, acc
        x.state[i](j) = state(i * 3 + j);
  }

  void TrajectoryTracker::hostRobotOdomCallback(const nav_msgs::OdometryConstPtr& msg){
    host_robot_odom_ = *msg;
  }

  void TrajectoryTracker::hostRobotImuCallback(const sensor_msgs::ImuConstPtr& msg){
    host_robot_imu_ = *msg;
    Eigen::Quaterniond q(host_robot_imu_.orientation.w,
                         host_robot_imu_.orientation.x,
                         host_robot_imu_.orientation.y,
                         host_robot_imu_.orientation.z);
    Eigen::Vector3d acc_b(host_robot_imu_.linear_acceleration.x,
                          host_robot_imu_.linear_acceleration.y,
                          host_robot_imu_.linear_acceleration.z);
    host_robot_acc_world_ = q * acc_b;
    host_robot_acc_world_(2) -= 9.8;
  }

  void TrajectoryTracker::hostRobotTaskCommandCallback(const std_msgs::UInt8 msg){
    if (msg.data == 5) // force switching to static state
      tracking_state_ = QUIT_TASK;
    else if (msg.data == 2)
      tracking_state_ = START_GRAPPING;
    else if (msg.data == 0)
      tracking_state_ = PRE_TRACKING;
    else if (msg.data == 6)
      tracking_state_ = CRUISE_TREASURE_BOX;
    else if (msg.data == 7)
      tracking_state_ = RELEASE_OBJECT;
    immediate_replan_flag_ = true;
  }
}
