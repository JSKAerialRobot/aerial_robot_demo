/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSK_MBZIRC_COMMON_MBZIRC_GAZEBO_UAV_COLLISION_RULE_PLUGIN_H
#define JSK_MBZIRC_COMMON_MBZIRC_GAZEBO_UAV_COLLISION_RULE_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <cmath>
#include <string>

namespace gazebo
{
class GazeboCollisionRule : public ModelPlugin
{
public:
  GazeboCollisionRule();
  virtual ~GazeboCollisionRule();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();

private:
  ros::NodeHandle* node_handle_;
  ros::ServiceClient motor_shutdown_client_;

  ros::CallbackQueue callback_queue_;
  event::ConnectionPtr update_connection_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  std::string link_name_;

  double init_z_offset_;
  bool terminated_;
  int takeoff_;

  // parameters
  std::string robot_name_space_;
};
}  // namespace gazebo

#endif  // JSK_MBZIRC_COMMON_MBZIRC_GAZEBO_UAV_COLLISION_RULE_PLUGIN_H
