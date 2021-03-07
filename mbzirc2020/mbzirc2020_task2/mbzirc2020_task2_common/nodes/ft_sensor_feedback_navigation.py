#!/usr/bin/env python

import math
import numpy as np

import rospy

from mbzirc2020_task2_common.hydrus_cooperation_interface import FTSensorFeedbackInterface

if __name__ == '__main__':
    rospy.init_node('ft_sensor_feedback_navigation')

    robot_ns = rospy.get_param('/ft_sensor_feedback_navigation/robot_ns','hydrus')
    sensor_takeoff = rospy.get_param('/ft_sensor_feedback_navigation/sensor_takeoff',False)
    sensor_landing = rospy.get_param('/ft_sensor_feedback_navigation/sensor_landing',False)

    hydrus = FTSensorFeedbackInterface(robot_ns=robot_ns,sensor_takeoff=sensor_takeoff,sensor_landing=sensor_landing)

    while not rospy.is_shutdown():

        hydrus.takeoff_landing_check()

        if not rospy.is_shutdown() and hydrus.ft_sensor_feedback_flag:

            #hydrus.set_yaw_free(flag=True)

            while not rospy.is_shutdown() and hydrus.ft_sensor_feedback_flag:

                hydrus.takeoff_landing_check()

                hydrus.ft_sensor_feedback_navigation()

                rospy.Rate(4).sleep()

            hydrus.hovering_on_the_spot()
            #hydrus.set_yaw_free(flag=False)
            rospy.Rate(60).sleep()

        rospy.Rate(60).sleep()

