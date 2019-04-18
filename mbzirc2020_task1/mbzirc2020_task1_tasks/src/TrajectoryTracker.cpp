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

    primitive_ = new MotionSinglePrimitive();

    pub_tracking_trajectory_ = nh_.advertise<nav_msgs::Path>("/tracking_path", 1);
    pub_tracking_target_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracking_target", 1);

    predictor_thread_ = boost::thread(boost::bind(&TrajectoryTracker::predictorThread, this));
    replan_timer_ = nh_.createTimer(ros::Duration(replan_timer_period_), &TrajectoryTracker::replanCallback, this);
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
    double period = 1.0;
    Eigen::VectorXd cur_state = object_trajectory_predictor_->getPredictedState(0.0); // p_x, v_x, p_y, v_y
    Eigen::VectorXd cur_u = object_trajectory_predictor_->getPredictedControlInput(0.0); // a_x, a_y
    Eigen::VectorXd end_state = object_trajectory_predictor_->getPredictedState(period);
    Eigen::VectorXd end_u = object_trajectory_predictor_->getPredictedControlInput(period);
    Eigen::VectorXd cur_state_full(3 * 3);
    Eigen::VectorXd end_state_full(3 * 3);
    cur_state_full << cur_state(0), cur_state(1), cur_u(0), // x axis
      cur_state(2), cur_state(3), cur_u(1), // y axis
      cur_state(4), cur_state(5), cur_u(2); // z axis
    // todo
    cur_state_full(6) -= 2.0; // add offset in z axis
    end_state_full << end_state(0), end_state(1), end_u(0),
      end_state(2), end_state(3), end_u(1),
      end_state(4), end_state(5), end_u(2);
    convertToMPState(x_start_, cur_state_full);
    convertToMPState(x_end_, end_state_full);

    primitive_->init(period, x_start_, x_end_);
    visualizationPrimitive();
  }

  void TrajectoryTracker::visualizationPrimitive(){
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
    pub_tracking_trajectory_.publish(tracking_path);

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
}
