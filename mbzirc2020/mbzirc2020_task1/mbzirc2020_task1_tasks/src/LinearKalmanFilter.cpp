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

#include <mbzirc2020_task1_tasks/LinearKalmanFilter.h>
namespace linear_kalman_filter{
  LinearKalmanFilter::LinearKalmanFilter(ros::NodeHandle nh, ros::NodeHandle nhp, int state_dim, int u_dim, int z_dim){
    nh_ = nh;
    nhp_ = nhp;

    state_dim_ = state_dim;
    u_dim_ = u_dim;
    z_dim_ = z_dim;

    initVariables();
  }

  void LinearKalmanFilter::initPrioriState(Eigen::VectorXd &x, Eigen::MatrixXd &P_priori){
    x_priori_ = x;
    P_priori_ = P_priori;
  }

  void LinearKalmanFilter::update(){
    // update
    y_wave_k_ = z_k_ - H_k_ * x_priori_;
    S_k_ = R_k_ + H_k_ * P_priori_ * H_k_.transpose();
    K_k_ = P_priori_ * H_k_.transpose() * S_k_.inverse();
    x_post_ = x_priori_ + K_k_ * y_wave_k_;
    Eigen::MatrixXd I_K_H(state_dim_, state_dim_);
    I_K_H = Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K_k_ * H_k_;
    P_post_ = I_K_H * P_priori_;

    // predict
    x_priori_ = F_k_ * x_post_ + B_k_ * u_k_;
    P_priori_ = F_k_ * P_post_ * F_k_.transpose() + Q_k_;

  }

  void LinearKalmanFilter::setModel(Eigen::VectorXd &z, Eigen::VectorXd &u, Eigen::MatrixXd &cur_F, Eigen::MatrixXd &cur_B, Eigen::MatrixXd &cur_H, Eigen::MatrixXd &Q, Eigen::MatrixXd &R){
    z_k_ = z;
    u_k_ = u;
    F_k_ = cur_F;
    B_k_ = cur_B;
    H_k_ = cur_H;
    Q_k_ = Q;
    R_k_ = R;
  }

  void LinearKalmanFilter::initVariables(){
    F_k_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    B_k_ = Eigen::MatrixXd::Zero(state_dim_, u_dim_);
    H_k_ = Eigen::MatrixXd::Zero(z_dim_, state_dim_);
    P_priori_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    P_post_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    S_k_ = Eigen::MatrixXd::Zero(z_dim_, z_dim_);
    K_k_ = Eigen::MatrixXd::Zero(state_dim_, z_dim_);
    Q_k_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    R_k_ = Eigen::MatrixXd::Zero(z_dim_, z_dim_);

    x_priori_ = Eigen::VectorXd::Zero(state_dim_);
    x_post_ = Eigen::VectorXd::Zero(state_dim_);
    u_k_ = Eigen::VectorXd::Zero(u_dim_);
    z_k_ = Eigen::VectorXd::Zero(z_dim_);
    y_wave_k_ = Eigen::VectorXd::Zero(z_dim_);
  }

  Eigen::VectorXd LinearKalmanFilter::getPrioriState(){
    return x_priori_;
  }

  Eigen::VectorXd LinearKalmanFilter::getPosterioriState(){
    return x_post_;
  }
}
