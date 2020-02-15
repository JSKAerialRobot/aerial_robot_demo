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

import sys
import rospy
import time
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState

class task1SpecialLanding:
    def init(self):
        rospy.init_node('task1_spcial_landing', anonymous=True)
        self.__speicial_landing_sub = rospy.Subscriber("/task1_motion_state_machine/task1_special_landing", Empty, self.__specialLandingCallback)

        self.__hydrusx_joints_ctrl_pub = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)
        self.__land_pub = rospy.Publisher("/teleop_command/land", Empty,  queue_size=1)
        rospy.sleep(0.5)

        self.__hydrus_joints_ctrl_msg = JointState()
        self.__hydrus_joints_ctrl_msg.name.append("joint1")
        self.__hydrus_joints_ctrl_msg.name.append("joint3")
        self.__hydrus_joints_ctrl_msg.position.append(1.5)
        self.__hydrus_joints_ctrl_msg.position.append(1.5)

    def __specialLandingCallback(self, msg):
        self.__object_odom = msg
        self.__hydrusx_joints_ctrl_pub.publish(self.__hydrus_joints_ctrl_msg)
        self.__land_pub.publish(Empty())
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        task1_spcial_landing = task1SpecialLanding()
        task1_spcial_landing.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
