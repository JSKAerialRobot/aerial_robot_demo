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

#include <mbzirc_common/mbzirc_gazebo_treasure_plugin.h>
#include <string>

namespace gazebo
{

  GazeboTreasure::GazeboTreasure()
  {
    magnet_on_.data = false;
  }

  GazeboTreasure::~GazeboTreasure()
  {
    event::Events::DisconnectWorldUpdateBegin(update_connection_);

    node_handle_->shutdown();
    delete node_handle_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboTreasure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;
    world_ = _model->GetWorld();
    link_ = _model->GetLink();
    link_name_ = link_->GetName();
    namespace_.clear();

    static_object_ = false;
    last_time_ = world_->GetSimTime();
    terminated_ = false;

    treasure_state_ = TREASURE_CRUISE;

    // load parameters from sdf
    if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

    if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
      {
        link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
        link_ = _model->GetLink(link_name_);
      }

    if (_sdf->HasElement("staticObject") && _sdf->GetElement("staticObject")->GetValue())
      {
        static_object_ = _sdf->GetElement("staticObject")->Get<std::string>() == "true"?true:false;
      }

    if (!link)
      {
        ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
        return;
      }


    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

    node_handle_ = new ros::NodeHandle(namespace_);
    pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true);  // set latch true
    ros::NodeHandle param_handle(*node_handle_, "controller");

    //subscribe to the gazebo models
    gazebo_model_sub_ = node_handle_->subscribe("/gazebo/model_states",3,&GazeboTreasure::gazeboCallback,this);
    magnet_release_sub_ = node_handle_->subscribe("/mag_on",3,&GazeboTreasure::magnetCallback,this);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                                boost::bind(&GazeboTreasure::Update, this));

  }


  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboTreasure::Update()
  {
    if ( terminated_ )
      {
        return;
      }
    else
      {
        math::Pose pose_object = link_->GetWorldPose();  //the pose of the object
        for(int i = 0; i < this->gazebo_models_.name.size(); i++)
          {
            int pirate_id, guard_id;
            pirate_id = gazebo_models_.name.at(i).find("hydrusx");
            guard_id = gazebo_models_.name.at(i).find("hawk");
            if (treasure_state_ == TREASURE_CRUISE){
              if (pirate_id >= 0){
                // judge whether treasure is grubbed
                geometry_msgs::Pose pose_pirate = gazebo_models_.pose.at(i);
                if(fabs(pose_pirate.position.x - pose_object.pos.x)<0.2&&
                   fabs(pose_pirate.position.y - pose_object.pos.y)<0.2&&
                   fabs(pose_pirate.position.z - pose_object.pos.z)<0.2){
                  treasure_state_ = TREASURE_CAPTURED;
                  updateTreasureState(i, -0.27);
                  break;
                }
              }
              else if (guard_id >= 0){
                // reset object positon
                updateTreasureState(i, -0.27);
                break;
              }
              else
                continue;
            }
            else if (treasure_state_ == TREASURE_CAPTURED){
              if (pirate_id >= 0){
                // reset object positon
                updateTreasureState(i, -0.17);
              }
              else
                continue;
            }
            else
              continue;
          }

        last_time_ = world_->GetSimTime();

      }

    if( static_object_ )
      {
        return;
      }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Reset the controller
  void GazeboTreasure::Reset()
  {
    state_stamp_ = ros::Time();
  }

  void GazeboTreasure::updateTreasureState(int owner_id, double offset_z){
    geometry_msgs::Pose pose = gazebo_models_.pose.at(owner_id);
    pose.position.z += offset_z;
    model_->SetLinkWorldPose(math::Pose(pose.position.x,
                                        pose.position.y,
                                        pose.position.z,
                                        0, 0, 0), link_);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboTreasure)

}  // namespace gazebo
