#!/usr/bin/env python

import math
import numpy as np

import rospy

from mbzirc2020_task2_common.hydrus_cooperation_interface import FTSensorFeedbackInterface

if __name__ == '__main__':
    rospy.init_node('ft_sensor_feedback_navigation')

    robot_ns = rospy.get_param('~robot_ns','hydrus')
    sensor_takeoff = rospy.get_param('~ft_sensor_feedback_navigation/sensor_takeoff',False)
    sensor_landing = rospy.get_param('~ft_sensor_feedback_navigation/sensor_landing',False)

    fb_interface = FTSensorFeedbackInterface(robot_ns=robot_ns,sensor_takeoff=sensor_takeoff,sensor_landing=sensor_landing)

    while not rospy.is_shutdown():

        fb_interface.takeoff_landing_check()

        if not rospy.is_shutdown() and fb_interface.ft_sensor_feedback_flag:

            #fb_interface.set_yaw_free(flag=True)

            while not rospy.is_shutdown() and fb_interface.ft_sensor_feedback_flag:

                fb_interface.takeoff_landing_check()

                fb_interface.ft_sensor_feedback_navigation()

                rospy.Rate(10).sleep()

            #fb_interface.hovering_on_the_spot()
            #fb_interface.set_yaw_free(flag=False)
            rospy.Rate(60).sleep()

        rospy.Rate(60).sleep()

