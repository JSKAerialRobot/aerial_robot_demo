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

import sys, select, termios, tty

msg = """

    g: start gripping the object
    r: force reset treasure to guard uav and quit the task
    s: pirate robot starts to track
    q: pirate robot quits tracking and keeps still

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

	rospy.init_node('task_command', anonymous=True)
        treasure_init_pub = rospy.Publisher("/treasure_force_init_cmd", Empty, queue_size=1)
        track_task_command_pub = rospy.Publisher('/track/task/command', UInt8, queue_size=1)
	task_cmd_msg = UInt8() 
	empty_msg = Empty() 

	try:
		while(True):
			key = getKey()
			# takeoff and landing
			if key == 'g':
                            task_cmd_msg.data = 1
			    track_task_command_pub.publish(task_cmd_msg)
                            print "[gripping command sent to pirate robot]"
			if key == 'r':
                            task_cmd_msg.data = 0
			    track_task_command_pub.publish(task_cmd_msg)
                            treasure_init_pub.publish(empty_msg)
                            print "[treause force reset and quit the task]"
			if key == 'q':
                            task_cmd_msg.data = 0
			    track_task_command_pub.publish(task_cmd_msg)
                            print "[quit the task]"
			if key == 's':
                            task_cmd_msg.data = 2
			    track_task_command_pub.publish(task_cmd_msg)
                            print "[tracking command sent to pirate robot]"
                        if (key == '\x03'):
			    break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


