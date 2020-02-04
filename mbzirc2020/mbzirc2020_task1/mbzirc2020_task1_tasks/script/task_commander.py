#!/usr/bin/env python

""" move hydrus around """

import sys
import time
import rospy
import math

from std_msgs.msg import Empty
from std_msgs.msg import Int8

class TaskCommander():
    def __init__(self, name="task_commander"):
        # init node
        # rospy.init_node(name)

        # define publisher
        self.reactive_motion_state_pub = rospy.Publisher("/reactive_motion/state", Int8, queue_size=1)

        # constants
        self.WAIT_TIME = 0.5

    def prepare_estimation(self):
        # send arm and takeoff
        time.sleep(self.WAIT_TIME)
        msg = Int8()
        msg.data = 1 ## WAITING
        self.reactive_motion_state_pub.publish(msg)
        time.sleep(self.WAIT_TIME)

    def stop_estimation(self):
        # send arm and takeoff
        time.sleep(self.WAIT_TIME)
        msg = Int8()
        msg.data = 0 ## STILL
        self.reactive_motion_state_pub.publish(msg)
        time.sleep(self.WAIT_TIME)

