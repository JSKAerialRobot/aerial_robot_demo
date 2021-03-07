#!/usr/bin/env python

import random
import math

import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from std_srvs.srv import Empty as EmptyService

from mbzirc2020_task2_common.hydrus_cooperation_interface import HydrusInterface

if __name__=='__main__':
    rospy.init_node('acc_mode_demo')

    hydrus = HydrusInterface(robot_ns='hydrus')
    rospy.sleep(1)

    hydrus.start()
    rospy.sleep(1)

    hydrus.set_position_ctrl_mode()
    rospy.sleep(1)

    hydrus.takeoff()
    rospy.sleep(1)

    hydrus.wait_for_hovering()
    rospy.sleep(1)

    while True:

        ### set yaw ###
        target_yaw = round(2*math.pi*random.random(),5)

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = target_yaw
        hydrus.uav_nav_pub.publish(nav_msg)

        rospy.loginfo(target_yaw)

        rospy.sleep(10)

        ### set acceleration ###
        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        nav_msg.target_acc_x = 0.5
        nav_msg.target_acc_y = 0.5
        hydrus.uav_nav_pub.publish(nav_msg)

        rospy.sleep(3)

        ### hovering ###
        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_x = 0.0
        nav_msg.target_vel_y = 0.0
        hydrus.uav_nav_pub.publish(nav_msg)

        rospy.sleep(1)

    
    rospy.spin()
    


