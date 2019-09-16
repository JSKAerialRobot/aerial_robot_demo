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

#include <mbzirc_common/mbzirc_gazebo_uav_collision_rule_plugin.h>
#include <string>

namespace gazebo
{

GazeboCollisionRule::GazeboCollisionRule()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboCollisionRule::~GazeboCollisionRule()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboCollisionRule::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();

  // default parameters
  robot_name_space_.clear();

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace"))
    robot_name_space_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(robot_name_space_);

  motor_shutdown_client_ = node_handle_->serviceClient<std_srvs::Empty>("shutdown");

  terminated_ = false;
  takeoff_ = -1;
  init_z_offset_ = -1;

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCollisionRule::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboCollisionRule::Update()
{
  // handle callbacks
  callback_queue_.callAvailable();

  if (terminated_) return;

  gazebo::physics::RayShapePtr rayShape = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    world_->GetPhysicsEngine()->CreateShape("ray", gazebo::physics::CollisionPtr()));

  double dist;
  std::string entityName;
  math::Box box = model_->GetLink("base_link")->GetCollisionBoundingBox();  // shoud specify to link?
  math::Vector3 start = model_->GetLink("base_link")->GetWorldPose().pos;
  math::Vector3 end = start;

  if (init_z_offset_ < 0) init_z_offset_ = box.min.z;

  start.z = box.min.z - init_z_offset_;
  end.z -= 1000;
  rayShape->SetPoints(start, end);
  rayShape->GetIntersection(dist, entityName);

  if (entityName == "arena::link_arena::collision")
    {
      if (dist < 0.01)
        {
          if (takeoff_ == -1) takeoff_ = 0;

          if (takeoff_ == 1)
            {
              ROS_WARN("Collide with ground! Distance is %f", dist);
              terminated_ = true;

              std_srvs::Empty srv;
              if (motor_shutdown_client_.call(srv))
                ROS_WARN("Shutdown motors!");
              else
                ROS_ERROR("Failed to call motors shutdown service");
            }
        }
      else
        {
          if (takeoff_ == 0 && dist > 0.5)
            {  // takeoff height should higher than 0.5m
              takeoff_ = 1;
            }
        }
    }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboCollisionRule)

}  // namespace gazebo

