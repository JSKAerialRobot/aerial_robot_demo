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

#include <mbzirc2020_task3_motion/open_form_planner.h>

OpenFormPlanner::OpenFormPlanner(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
  {
    /* rosparam init */
    rosParamInit();

    /* initialize the multilink kinematics */
    kinematics_ = std::make_unique<HydrusRobotModel>(true);

    /* hard coding for quad model */
    if (kinematics_->getRotorNum() != 4)
      {
        ROS_WARN("This is not quad model: %d ", (int)kinematics_->getRotorNum());
        return;
      }

    for(int i = 1; i < 4; i++)
      {
        open_form_.name.push_back(std::string("joint")+std::to_string(i));
        open_form_.position.push_back(M_PI/2);
      }
    kinematics_->updateRobotModel(open_form_);
    auto seg_tf_mag = kinematics_->getSegmentsTf();

    /* add sheet as extra module */
    if(as_extra_module_)
      {
        /* hard-coding*/

        auto anchor_point1_kdl = seg_tf_mag.at(std::string("link1")).Inverse() * seg_tf_mag.at(std::string("leg1"));
        kinematics_->addExtraModule(std::string("first_half_sheet_weight"), std::string("link1"), anchor_point1_kdl, KDL::RigidBodyInertia(sheet_mass_/2));
        auto anchor_point2_kdl = seg_tf_mag.at(std::string("link4")).Inverse() * seg_tf_mag.at(std::string("leg5"));
        kinematics_->addExtraModule(std::string("second_half_sheet_weight"), std::string("link4"), anchor_point2_kdl, KDL::RigidBodyInertia(sheet_mass_/2));

        std::string srv_name;
        nhp_.param("add_extra_moudle_srv_name", srv_name, std::string("add_extra_module"));
        if(ros::service::waitForService(srv_name, 5000))
          {
            auto client = nh_.serviceClient<aerial_robot_model::AddExtraModule>(srv_name);

            aerial_robot_model::AddExtraModule srv;
            srv.request.action = aerial_robot_model::AddExtraModule::Request::ADD;
            srv.request.module_name = std::string("first_half_sheet_weight");
            srv.request.parent_link_name = std::string("link1");
            srv.request.transform = tf2::kdlToTransform(anchor_point1_kdl).transform;
            srv.request.inertia.m = sheet_mass_/2;

            if (client.call(srv))
              {
                ROS_DEBUG_STREAM("Succeed to add extra module for: " << srv.request.module_name);
              }
            else
              {
                ROS_ERROR_STREAM("Failed to call service " << srv_name);
                return;
              }

            srv.request.module_name = std::string("second_half_sheet_weight");
            srv.request.parent_link_name = std::string("link4");
            srv.request.transform = tf2::kdlToTransform(anchor_point2_kdl).transform;

            if (client.call(srv))
              {
                ROS_DEBUG_STREAM("Succeed to add extra module for: " << srv.request.module_name);
              }
            else
              {
                ROS_ERROR_STREAM("Failed to call service " << srv_name);
                return;
              }
          }
        else
          {
            ROS_ERROR_STREAM("Cannot find service name called " << srv_name);
            return;
          }
      }

    /* do planning */
    if(!plan())
      {
        ROS_ERROR_STREAM("Invalid planning");
        return;
      }

#if 0 // debug: add sheet as extra module
    /* wait for service */
#endif

    /* advertise */
    std::string topic_name;
    nhp_.param("joint_ctrl_topic_name", topic_name, std::string("joints_ctrl"));
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);

    /* service server */
    send_open_form_service_ = nh_.advertiseService(std::string("send_open_form"), &OpenFormPlanner::sendOpenFormCallback, this);
  }

bool OpenFormPlanner::sendOpenFormCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  joint_pub_.publish(open_form_);
  res.success = true;
}

void OpenFormPlanner::rosParamInit()
{
  nhp_.param("verbose", verbose_, false);
  nhp_.param("debug", debug_, false);

  nhp_.param("as_extra_module", as_extra_module_, false); // add sheet as extra module
  nhp_.param("sheet_mass", sheet_mass_, 0.3); //Kg
  nhp_.param("sheet_width", sheet_width_, 1.0); //m
}

bool OpenFormPlanner::plan()
{
  double link_length = kinematics_->getLinkLength();
  double min_hovering_force_diff = 1e6;
  double delta_angle = 1e-3;
  double angle_bound = M_PI / 2;// can get from model
  Eigen::VectorXd optimal_hovering_force_vec;

  bool oneshot_search;
  double j2_angle;
  nhp_.param("oneshot_search", oneshot_search, false);
  nhp_.param("serach_init_angle", j2_angle, M_PI/2);

  /* debug */
  std::string topic_name;
  nhp_.param("joint_ctrl_topic_name", topic_name, std::string("joints_ctrl"));
  ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);

  while (1)
    {
      auto candidate_form = open_form_;

      /* calculate the j1 (== j3) from the geometry  */
      double a = 2 * link_length * sin((M_PI - j2_angle)/2);
      double phi = asin((sheet_width_ - a ) / 2 / link_length);
      double j1_angle = M_PI/2 - phi - j2_angle /2;

      if(j1_angle > angle_bound) break;

      /* assign the joint angles */
      candidate_form.position.at(0) = j1_angle;
      candidate_form.position.at(1) = j2_angle;
      candidate_form.position.at(2) = j1_angle;

      /* update robot model */
      kinematics_->updateRobotModel(candidate_form);
      kinematics_->modelling();

      double hovering_force_diff = kinematics_->getOptimalHoveringThrust().maxCoeff() - kinematics_->getOptimalHoveringThrust().minCoeff();

      if(hovering_force_diff < min_hovering_force_diff)
        {
          optimal_hovering_force_vec = kinematics_->getOptimalHoveringThrust();
          min_hovering_force_diff = hovering_force_diff;
          open_form_ = candidate_form;
        }

        if(debug_)
        {
          ros::Duration(0.1).sleep();
          joint_pub.publish(candidate_form);
          auto seg_tf_mag = kinematics_->getSegmentsTf();
          auto width = (seg_tf_mag.at(std::string("leg1")).p - seg_tf_mag.at(std::string("leg5")).p).Norm();
          ROS_INFO("joints: [%f, %f, %f], width: %f, a: %f, phi: %f", j1_angle, j2_angle, j1_angle, width, a, phi);
          std::cout << "the hovering force is \n" << kinematics_->getOptimalHoveringThrust().transpose() << std::endl;
        }

      j2_angle -= delta_angle;

      if(oneshot_search) break;
    }

  if(verbose_)
    {
      ROS_INFO("the optimal openning form is: [%f, %f, %f], the min force vec is %f",
               open_form_.position.at(0),
               open_form_.position.at(1),
               open_form_.position.at(2),
               min_hovering_force_diff);
      std::cout << "the optimal hovering force is \n" << optimal_hovering_force_vec.transpose() << std::endl;
    }


  return true;
}
