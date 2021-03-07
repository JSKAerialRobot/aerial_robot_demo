#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int8
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry

class Master:
    def __init__(self,robot_ns):

        self.uav_nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=10)

        self.slave_ft_sensor_feedback_switch_pub = rospy.Publisher('/hydrus2/ft_sensor_feedback_switch', Int8, queue_size=150)

        self.slave_uav_nav_pub = rospy.Publisher('/hydrus2/uav/nav', FlightNav, queue_size=10)

        self.odom_sub = rospy.Subscriber(robot_ns +'/uav/cog/odom', Odometry, self.OdometryCallback)

        self.position = np.array([0,0,0])

    def OdometryCallback(self,msg):

        self.position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])

    def slave_hovering(self):

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        nav_msg.target_acc_x = 0
        nav_msg.target_acc_y = 0
        self.slave_uav_nav_pub.publish(nav_msg)

        rospy.sleep(3)


    def X_axis_movement(self):

        position = self.position

        pos = np.linspace(0,0.3,30)

        for p in pos:
            nav_msg = FlightNav()
            nav_msg.header.frame_id='global'
            # LOCAL
            nav_msg.control_frame=0
            # COG
            nav_msg.target=1
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = position[0]+p
            nav_msg.target_pos_y = position[1]
            self.uav_nav_pub.publish(nav_msg)

            rospy.sleep(0.1)

        print('nav 1')

        pos = np.linspace(0.3,-0.3,60)

        for p in pos:
            nav_msg = FlightNav()
            nav_msg.header.frame_id='global'
            # LOCAL
            nav_msg.control_frame=0
            # COG
            nav_msg.target=1
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = position[0]+p
            nav_msg.target_pos_y = position[1]
            self.uav_nav_pub.publish(nav_msg)

            rospy.sleep(0.1)

        print('nav 2')

        pos = np.linspace(-0.3,0,30)

        for p in pos:
            nav_msg = FlightNav()
            nav_msg.header.frame_id='global'
            # LOCAL
            nav_msg.control_frame=0
            # COG
            nav_msg.target=1
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = position[0]+p
            nav_msg.target_pos_y = position[1]
            self.uav_nav_pub.publish(nav_msg)

            rospy.sleep(0.1)

        print('nav 3')

if __name__=="__main__":

    rospy.init_node('master_X_axis_movement')

    robot_ns_master = 'hydrus1'

    master = Master(robot_ns=robot_ns_master)

    rospy.sleep(1)

    master.slave_hovering()

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 1
    master.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)

    rospy.sleep(1)

    master.X_axis_movement()

    master.X_axis_movement()

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 0
    master.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)
