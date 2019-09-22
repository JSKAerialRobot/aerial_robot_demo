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

#include <mbzirc2020_task1_tasks/MotionSinglePrimitive.h>
namespace motion_single_primitive{
  MotionSinglePrimitive::MotionSinglePrimitive(){

  }

  void MotionSinglePrimitive::init(double period, MPState x_start, MPState x_end){
    polynomial_order_ = 6;
    state_dim_ = x_start.state.size();
    traj_poly_param_vec_.resize(state_dim_);
    period_ = period;
    x_start_ = x_start;
    x_end_ = x_end;
    for (int i = 0; i < state_dim_; ++i){
      Eigen::Vector3d state_delta;
      state_delta(0) = x_end.state[i](0) - x_start.state[i](0) - x_start.state[i](1) * period_ - x_start.state[i](2) * pow(period_, 2) / 2.0;
      state_delta(1) = x_end.state[i](1) - x_start.state[i](1) - x_start.state[i](2) * period_;
      state_delta(2) = x_end.state[i](2) - x_start.state[i](2);
      Eigen::Vector3d state_delta_dash;
      for (int i = 0; i < 3; ++i)
        state_delta_dash(i) = state_delta(2 - i);
      Eigen::Matrix3d mat_dash;
      mat_dash << 60 * pow(period, 2), -360 * period, 720,
        -24 * pow(period, 3), 168 * pow(period, 2), -360 * period,
        3 * pow(period, 4), -24 * pow(period, 3), 60 * pow(period, 2);
      Eigen::Vector3d params_dash = mat_dash / pow(period, 5) * state_delta_dash;
      Eigen::VectorXd traj_poly_param = Eigen::VectorXd::Zero(polynomial_order_);
      traj_poly_param << x_start.state[i](0), x_start.state[i](1), x_start.state[i](2) / 2.0,
        params_dash(2) / 6.0, params_dash(1) / 24.0, params_dash(0) / 120.0;
      traj_poly_param_vec_[i] = traj_poly_param;
    }
  }

  double MotionSinglePrimitive::getPrimitiveEnergy(int derivative_order){
    double energy_sum = 0.0; // default is snap energy

    /* method 1: only calculate the jerk square as trajectory cost */
    for (int i1 = derivative_order; i1 < polynomial_order_; ++i1){
      int factor1 = nMultiply(i1, derivative_order);
      for (int i2 = derivative_order; i2 < polynomial_order_; ++i2){
        int factor2 = nMultiply(i2, derivative_order);
        double factor = factor1 * factor2 / (i1 + i2 - 2 * derivative_order + 1)
            * pow(period_, i1 + i2 - 2 * derivative_order + 1);
        for (int j = 0; j < traj_poly_param_vec_.size(); ++j)
          energy_sum += factor * traj_poly_param_vec_[j](i1)
            * traj_poly_param_vec_[j](i2);
      }
    }

    /* method 2: follow Mueller's cost function */
    // for (int i = 0; i < traj_poly_param_vec_.size(); ++i){
    //   double a = traj_poly_param_vec_[i](polynomial_order_ - 1) * 120.0;
    //   double b = traj_poly_param_vec_[i](polynomial_order_ - 2) * 24.0;
    //   double c = traj_poly_param_vec_[i](polynomial_order_ - 3) * 6.0;
    //   energy_sum += pow(c, 2) * period_
    //     + b * c * pow(period_, 2)
    //     + (pow(b, 2) + a * c) * pow(period_, 3) / 3.0
    //     + a * b * pow(period_, 4) / 4.0
    //     + pow(a, 2) * pow(period_, 5) / 20.0
    //     ;
    // }
    return energy_sum;
  }

  MPState MotionSinglePrimitive::getWaypointState(double relative_time){
    MPState waypoint_state;
    for (int i = 0; i < state_dim_; ++i)
      waypoint_state.state.push_back(Eigen::Vector3d::Zero());
    for (int i = 0; i < 3; ++i){ // pos, vel, acc
        Eigen::VectorXd state_vec = calculatePolynomialValue(relative_time, i);
      for (int j = 0; j < state_dim_; ++j)
        waypoint_state.state[j](i) = state_vec[j];
      }
    return waypoint_state;
  }

  Eigen::VectorXd MotionSinglePrimitive::calculatePolynomialValue(double relative_time, int derivative_order){
    Eigen::VectorXd result = Eigen::VectorXd::Zero(state_dim_);
    for (int l = 0; l < state_dim_; ++l)
      for (int i = derivative_order; i < polynomial_order_; ++i)
        result[l] += traj_poly_param_vec_[l](i) * pow(relative_time, i - derivative_order) * nMultiply(i, derivative_order);
    return result;
  }

  int MotionSinglePrimitive::nMultiply(int order, int derivative_order){
    int result = 1;
    for (int i = order; i > order - derivative_order; --i)
      result *= i;
    return result;
  }
}
