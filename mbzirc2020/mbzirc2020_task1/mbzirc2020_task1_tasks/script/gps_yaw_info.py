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

import rospy

from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from spinal.msg import Gps, Imu

class GpsYawInfo(object):
        def __init__(self):
                rospy.init_node("gps_yaw_info")
                self.gps_sub = rospy.Subscriber("/gps", Gps, self.gpsCallback)
                self.imu_sub = rospy.Subscriber("/imu", Imu, self.imuCallback)
                self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCallback)
                self.flight_state_sub = rospy.Subscriber("/flight_state", UInt8, self.stateCallback)

                self.latitude = 0
                self.longitude = 0
                self.yaw = 0
                self.joy = False

        def joyCallback(self, msg):
                self.joy = True

        def gpsCallback(self, msg):
                self.latitude = msg.location[0]
                self.longitude = msg.location[1]

        def imuCallback(self, msg):
                self.yaw = msg.angles[2]

        def stateCallback(self, msg):
                if msg.data == 0: # idle
                        rospy.loginfo_throttle(1.0, "gps: [" + str(self.latitude) + ", " + str(self.longitude) + ", yaw: " + str(self.yaw) + "], joy:" + str(self.joy))

        def spin(self):
                rospy.spin()

if __name__ == "__main__":
    GpsYawInfo().spin()


