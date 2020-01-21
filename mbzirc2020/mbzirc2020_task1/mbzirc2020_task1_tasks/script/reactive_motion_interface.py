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

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from std_msgs.msg import Bool

import sys, select, termios, tty

msg = """

    0: keep still
    1: keep waiting
    3: stop trackig

    please don't have caps lock on.
    CTRL+c to quit
"""

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	print msg

	rospy.init_node('reactive_motion_interface', anonymous=True)
        motion_state_pub = rospy.Publisher("/reactive_motion/state", Int8, queue_size=1)
	state_cmd_msg = Int8()

        ## STILL 0
        ## WAITING 1
        ## TRACKING 2
        ## STOP_TRACKING 3
	try:
		while(True):
			key = getKey()
			# takeoff and landing
			if key == '0':
                            state_cmd_msg.data = 0
			    motion_state_pub.publish(state_cmd_msg)
                            print "[Robot keeps still]"
			if key == '1':
                            state_cmd_msg.data = 1
			    motion_state_pub.publish(state_cmd_msg)
                            print "[Robot keeps waiting]"
			if key == '3':
                            state_cmd_msg.data = 3
			    motion_state_pub.publish(state_cmd_msg)
                            print "[Robot stops tracking]"
                        if (key == '\x03'):
			    break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


