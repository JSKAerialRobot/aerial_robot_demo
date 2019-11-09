#!/usr/bin/env python
# from __future__ import print_function
import sys
import rospy
import time
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from std_msgs.msg import String
from aerial_robot_msgs.msg import FlightNav

import numpy as np
import sys

__author__ = "kuromiya@jsk.imi.i.u-tokyo.ac.jp (Naoki Kuromiya)"


class flying_controller():
    def __init__(self):

        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")

        #  publishers
        self.publish_flight = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.message_pub = rospy.Publisher("/message2", String, queue_size=1)
        
        #  subscribers
        self.message_sub = rospy.Subscriber("/message1", String, self.messageCallback, queue_size=1, buff_size=1)
        self.target_sub = rospy.Subscriber("/target_object/pos", Vector3Stamped, self.targetCallback, queue_size=1, buff_size=1)
        self.target_angle_sub = rospy.Subscriber("/target_object/angle", Float64, self.targetAngleCallback, queue_size=1, buff_size=1)

        self.angle = 0.0
        self.WAIT = 0.5
        self.goposWAIT = 10

        self.uav_xyz_pos = np.array([0.0, 0.0, 0.0])
        self.tar_xyz_pos = np.array([0.0, 0.0, 0.0])
        
        self.message = ""
        self.phase = 0
        #timer
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        
        rospy.Timer(rospy.Duration(0.5), self.controllCallback)
        
    def goPos(self, x, y, z):
        rospy.loginfo("Go to Destination")
        msg = FlightNav()
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.POS_MODE

        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.BASELINK
        msg.target_pos_x = x
        msg.target_pos_y = y
        msg.target_pos_z = z + 1.0
        time.sleep(self.WAIT)
        self.publish_flight.publish(msg)
        time.sleep(self.goposWAIT)


    def uavOdomCallback(self, msg):
        self.uav_xyz_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_yaw = rpy[2]

        
    def messageCallback(self, msg):
        self.message = msg.data
        
    def controllCallback(self, event):
        if self.phase == 1:
            msg = String()
            msg.data = "going"
            self.message_pub.publish(msg)

            count = 0
            for a, b in zip(self.uav_xyz_pos, self.tar_xyz_pos):
                if abs(a - b) < 0.10:
                    count += 1
                
            if count >= 2:
                self.phase = 2
                

        elif self.phase == 2:
             msg = String()
             msg.data = "blick reached"
             self.message_pub.publish(msg)
             print("reached!")


    def targetCallback(self, msg):
        if self.message == "red object detected" and self.phase == 0:
            self.goPos(msg.vector.x, msg.vector.y, msg.vector.z);
            self.phase = 1
            
            self.tar_xyz_pos = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        
            
    def targetAngleCallback(self, msg):
        self.angle = msg.data

    def workingfhaseCallback(self, msg):
        self.working_fhase = msg.data



if __name__ == "__main__":
    rospy.init_node("task2_gopos", anonymous=True)
    controller = flying_controller()
    rospy.spin()
