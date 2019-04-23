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

#ifndef TRAJECTORY_TRACKER_H
#define TRAJECTORY_TRACKER_H

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>

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

/* thread */
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

/* local */
#include <mbzirc2020_task1_tasks/TrajectoryPredictor.h>
#include <mbzirc2020_task1_tasks/MotionSinglePrimitive.h>
#include <mbzirc2020_task1_tasks/PrimitiveParams.h>
using namespace trajectory_predictor;
using namespace motion_single_primitive;

namespace trajectory_tracker{
  #define PRE_TRACKING 0
  #define KEEP_TRACKING 1
  #define START_GRAPPING 2
  #define IN_GRAPPING 3
  #define KEEP_STILL 4
  #define QUIT_TASK 5
  class TrajectoryTracker{
  public:
    TrajectoryTracker(ros::NodeHandle nh, ros::NodeHandle nhp);

  private:
    /* basic */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Timer replan_timer_;
    double replan_prev_time_;
    double replan_timer_period_;
    MotionSinglePrimitive *primitive_;
    int tracking_state_;
    double primitive_period_;
    bool immediate_replan_flag_;

    double kf_predict_horizon_;
    boost::thread predictor_thread_;
    boost::mutex mutex_;
    TrajectoryPredictor *object_trajectory_predictor_;
    MPState x_start_;
    MPState x_end_;

    // host robot
    nav_msgs::Odometry host_robot_odom_;
    sensor_msgs::Imu host_robot_imu_;
    Eigen::Vector3d host_robot_acc_world_;

    ros::Publisher pub_tracking_trajectory_;
    ros::Publisher pub_tracking_target_markers_;
    ros::Publisher pub_tracking_primitive_params_;

    ros::Subscriber sub_host_robot_odom_;
    ros::Subscriber sub_host_robot_imu_;
    ros::Subscriber sub_host_robot_task_command_;

    void predictorThread();
    void replanImpl();
    void replanCallback(const ros::TimerEvent& event);
    void convertToMPState(MPState &x, Eigen::VectorXd state);
    void publishPrimitiveParam();
    void visualizationPrimitive();
    void hostRobotOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void hostRobotImuCallback(const sensor_msgs::ImuConstPtr& msg);
    void hostRobotTaskCommandCallback(const std_msgs::UInt8 msg);
  };
}

#endif
