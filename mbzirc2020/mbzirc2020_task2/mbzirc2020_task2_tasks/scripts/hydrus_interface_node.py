#!/usr/bin/env python

import math
import numpy as np

import rospy

from task2_hydrus_interface import Task2HydrusInterface

if __name__ == '__main__':
    rospy.init_node('hydrus_interface_node')

    robot_ns = rospy.get_param('~robot_ns','hydrus')

    hydrus = Task2HydrusInterface(robot_ns=robot_ns)

    rospy.spin()
