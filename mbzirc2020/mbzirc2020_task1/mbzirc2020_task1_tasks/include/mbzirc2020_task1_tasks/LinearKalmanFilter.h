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

#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

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

namespace linear_kalman_filter{
  class LinearKalmanFilter{
  public:
    LinearKalmanFilter(ros::NodeHandle nh, ros::NodeHandle nhp, int state_dim, int u_dim, int z_dim);
    void initPrioriState(Eigen::VectorXd &x, Eigen::MatrixXd &P_priori);
    void update();
    void setModel(Eigen::VectorXd &z, Eigen::VectorXd &u, Eigen::MatrixXd &cur_F, Eigen::MatrixXd &cur_B, Eigen::MatrixXd &cur_H, Eigen::MatrixXd &Q, Eigen::MatrixXd &R);
    Eigen::VectorXd getPrioriState();
    Eigen::VectorXd getPosterioriState();

  private:
    /* basic */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    int state_dim_;
    int u_dim_;
    int z_dim_; // observation value dimension

    double kf_freq_;
    Eigen::MatrixXd F_k_;
    Eigen::MatrixXd B_k_;
    Eigen::MatrixXd H_k_;
    Eigen::MatrixXd P_priori_;
    Eigen::MatrixXd P_post_;
    Eigen::MatrixXd S_k_;
    Eigen::MatrixXd K_k_;
    Eigen::MatrixXd Q_k_;
    Eigen::MatrixXd R_k_;

    Eigen::VectorXd x_priori_;
    Eigen::VectorXd x_post_;
    Eigen::VectorXd u_k_;
    Eigen::VectorXd z_k_; // observation
    Eigen::VectorXd y_wave_k_; // residual

    void initVariables();
  };
}

#endif
