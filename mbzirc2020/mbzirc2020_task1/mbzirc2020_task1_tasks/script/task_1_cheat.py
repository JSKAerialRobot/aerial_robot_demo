#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2018, JSK Robotics Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs.srv import EnableMotors
from aerial_robot_msgs.msg import FlightNav

__author__ = 'shifan@jsk.imi.i.u-tokyo.ac.jp (Fan Shi)'

if __name__ == '__main__':
    rospy.init_node('task_1_cheat', anonymous=True)
    hawk_pose_cmd_pub = rospy.Publisher("/hawk/command/pose", PoseStamped, queue_size=1)
    hydrus_start_pub = rospy.Publisher('/teleop_command/start', Empty, queue_size=1)
    hydrus_takeoff_pub = rospy.Publisher('/teleop_command/takeoff', Empty, queue_size=1)
    hydrus_nav_cmd_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
    rospy.sleep(1.0)
    ## hydrus start and takeoff
    hydrus_start_pub.publish(Empty())
    rospy.sleep(1.0)
    hydrus_takeoff_pub.publish(Empty())

    ## hawk enable motor
    rospy.wait_for_service('/hawk/enable_motors')
    try:
        enable_hawk_motor_srv = rospy.ServiceProxy('/hawk/enable_motors', EnableMotors)
        resp1 = enable_hawk_motor_srv(True)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed")

    rospy.loginfo("initalization finished")
    rospy.sleep(1.0)

    ## hawk fly to the fixed position
    hawk_pose_cmd_msg = PoseStamped()
    hawk_pose_cmd_msg.header.frame_id = "world"
    hawk_pose_cmd_msg.pose.position.x = 2.0
    hawk_pose_cmd_msg.pose.position.y = 2.0
    hawk_pose_cmd_msg.pose.position.z = 2.5
    hawk_pose_cmd_pub.publish(hawk_pose_cmd_msg)
    rospy.loginfo("guard robot flys to fixed position")
    rospy.sleep(5)
    rospy.loginfo("guard robot arrives")
    rospy.sleep(15)
    rospy.loginfo("pirate robot takeoffs")

    ## hydrus fly to the fixed position
    hydrus_cmd_msg = FlightNav()
    hydrus_cmd_msg.control_frame = hydrus_cmd_msg.WORLD_FRAME
    hydrus_cmd_msg.target = hydrus_cmd_msg.COG
    hydrus_cmd_msg.pos_xy_nav_mode = hydrus_cmd_msg.POS_MODE
    hydrus_cmd_msg.pos_z_nav_mode = hydrus_cmd_msg.POS_MODE
    hydrus_cmd_msg.psi_nav_mode = hydrus_cmd_msg.POS_MODE
    hydrus_cmd_msg.target_pos_x = 2.0
    hydrus_cmd_msg.target_pos_y = 2.0 - 0.6 * 0.707
    hydrus_cmd_msg.target_pos_z = 2.05
    hydrus_cmd_msg.target_psi = -0.7850
    hydrus_nav_cmd_pub.publish(hydrus_cmd_msg)
    rospy.loginfo("pirate robot flys to grub treasure")
    rospy.sleep(10)

    ## hydrus fly back to the safe position
    hydrus_cmd_msg.target_pos_x = 0.0
    hydrus_cmd_msg.target_pos_y = 0.0
    hydrus_cmd_msg.target_pos_z = 0.7
    hydrus_nav_cmd_pub.publish(hydrus_cmd_msg)
    rospy.loginfo("pirate robot flys to safe position")
    rospy.sleep(3)

    rospy.spin()
