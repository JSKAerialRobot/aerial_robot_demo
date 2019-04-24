#!/usr/bin/env python

""" move hydrus around """

import sys
import time
import rospy
import math

from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_srvs.srv import Trigger

class HydrusCommander():
    def __init__(self, name="hydrus_commander"):
        # init node
        # rospy.init_node(name)

        # define publisher
        self.start_pub = rospy.Publisher("/teleop_command/start", Empty, queue_size=1)
        self.nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.takeoff_pub = rospy.Publisher("/teleop_command/takeoff", Empty,  queue_size=1)
        self.extra_servos_ctrl_pub = rospy.Publisher("/hydrusx/extra_servos_ctrl", JointState, queue_size=1)
        self.joints_ctrl_pub = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)

        # constants
        self.WAIT_TIME = 0.5

    def arm_and_takeoff(self):
        # send arm and takeoff
        time.sleep(self.WAIT_TIME)
        self.start_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)
        self.takeoff_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def move_to(self, pos_x, pos_y):
        """move to target x, y position"""
        # send desired position
        nav_msg = FlightNav()

        nav_msg.target = nav_msg.COG
        nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE
        nav_msg.target_pos_x = pos_x
        nav_msg.target_pos_y = pos_y
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    def change_height(self, pos_z):
        """ change height """
        nav_msg = FlightNav()
        nav_msg.target = nav_msg.COG
        nav_msg.pos_z_nav_mode = nav_msg.POS_MODE
        nav_msg.target_pos_z = pos_z
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    def change_yaw(self, psi):
        """ change yaw """
        nav_msg = FlightNav()
        nav_msg.psi_nav_mode = nav_msg.POS_MODE
        nav_msg.target_psi = psi
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    def move_camera(self, angle_rad):
        """ move camera to angle_rad """
        joint_msg = JointState()
        joint_msg.position = [angle_rad]
        time.sleep(self.WAIT_TIME)
        self.extra_servos_ctrl_pub.publish(joint_msg)
        time.sleep(self.WAIT_TIME)

    def joint_publish(self, joint_state):
        """publish joint_state list"""
        joint_msg = JointState()
        joint_msg.position = joint_state
        time.sleep(self.WAIT_TIME)
        self.joints_ctrl_pub.publish(joint_msg)
        time.sleep(self.WAIT_TIME)

    def open_joints(self):
        """open links"""
        # self.joint_publish([1.1, 1.1, 1.1])
        rospy.wait_for_service("/send_open_form")
        try:
            send_open_form = rospy.ServiceProxy('/send_open_form', Trigger)
            resp1 = send_open_form()
            return
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

    def close_joints(self):
        """close links"""
        self.joint_publish([1.55, 1.55, 1.55])

    def covering_motion(self,pos_x, pos_y, cog_yaw, covering_pre_height, covering_post_height, covering_move_dist):
        dest_yaw = cog_yaw + 0.785 # correct forward angle of open form
        forward_dir = [-covering_move_dist*math.sin(dest_yaw), covering_move_dist*math.cos(dest_yaw)]
        self.open_joints()
        time.sleep(2)
        self.move_to(pos_x+forward_dir[0]/2, pos_y+forward_dir[1]/2)
        time.sleep(1)
        self.change_height(covering_pre_height)
        time.sleep(5)

        self.move_to(pos_x-forward_dir[0], pos_y-forward_dir[1])
        self.change_height(covering_post_height)

def main():
    rospy.init_node('hydrus_test')
    commander = HydrusCommander()
    # commander.arm_and_takeoff()
    # time.sleep(10)
    # commander.open_joints()
    # time.sleep(2)
    # commander.close_joints()

    commander.close_joints()
    commander.change_yaw(0.0)
    commander.change_height(1.0)
    commander.move_to(2, 2)
    time.sleep(5)
    commander.covering_motion(2,2,0,0.8, 0.3, 1.0)

if __name__ == "__main__":
    main()

