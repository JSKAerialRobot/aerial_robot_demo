// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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

#pragma once

/* ros */
#include <ros/ros.h>

/* kinematics model */
#include <hydrus/hydrus_robot_model.h>

/* ros msg & srv */
#include <std_srvs/Trigger.h>
#include <aerial_robot_model/AddExtraModule.h>

class OpenFormPlanner
{
public:
  OpenFormPlanner(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~OpenFormPlanner(){}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  std::unique_ptr<HydrusRobotModel> kinematics_;
  sensor_msgs::JointState open_form_;

  ros::Publisher joint_pub_;
  ros::ServiceServer send_open_form_service_;

  bool verbose_;
  bool debug_;
  bool as_extra_module_;
  double sheet_width_;

  bool sendOpenFormCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void rosParamInit();
  bool plan();

};
