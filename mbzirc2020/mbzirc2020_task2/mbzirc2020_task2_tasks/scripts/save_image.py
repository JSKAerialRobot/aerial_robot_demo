#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import time
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
import tf
import tf2_ros
import numpy as np
import cv2
from cv_bridge import CvBridge
from spinal.msg import FourAxisCommand
import pyclustering
from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
from copy import deepcopy

class image_saver():
    def __init__(self):
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")
        self.publish_start = rospy.Publisher("/teleop_command/start", Empty, queue_size=1)
        self.publihsh_takeoff = rospy.Publisher("/teleop_command/takeoff", Empty, queue_size=1)
        self.publish_flight = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.publish_joints_ctrl = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)
        self.publish_extra_joints_ctrl = rospy.Publisher("/hydrusx/extra_servos_ctrl", JointState, queue_size=1)
        self.publish_image = rospy.Publisher("/image_center", Image, queue_size=1)
        self.task_start = False
        self.task_number = 0
        self.image_count = 0
        self.distance_count = 0

        #  subscribers

        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        self.image_sub_ = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.imageCallback, queue_size=1, buff_size=2)

    def uavOdomCallback(self, msg):
        self.uav_xyz_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_yaw_ = rpy[2]

    def imageCallback(self, msg):
        new_msg = FlightNav()
        camera_angle = JointState()

        new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.header.stamp = rospy.Time.now()
        new_msg.control_frame = FlightNav.WORLD_FRAME
        new_msg.target = FlightNav.COG
        new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.target_pos_x = self.uav_xyz_pos[0]
        new_msg.target_pos_y = self.uav_xyz_pos[1]
        new_msg.target_pos_z = self.uav_xyz_pos[2]

        bridge = CvBridge()
        #  imgmsg -> ndarray
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        name = "/home/kuromiya/Pictures/red_stack/distance/" + str(self.distance_count) + "/" + str(self.image_count) + ".png"
        cv2.imwrite(name, image)
        save_msg = str(self.distance_count) + "-" + str(self.image_count) + " saved!"
        rospy.loginfo(save_msg)
        new_msg.target_psi = self.uav_yaw_ + 0.10
        new_msg.psi_nav_mode = FlightNav.POS_MODE
        self.publish_flight.publish(new_msg)
        # time.sleep(2)
        self.image_count += 1
        if self.image_count > 15:
            new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_pos_x = 0.0
            new_msg.target_pos_y = self.uav_xyz_pos[1] + 0.50
            new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            self.publish_flight.publish(new_msg)
            time.sleep(10)
            new_msg.psi_nav_mode = FlightNav.POS_MODE
            new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_psi = -0.5
            self.publish_flight.publish(new_msg)
            time.sleep(10)
            self.image_count = 0
            self.distance_count += 1
        if self.distance_count > 30:
            rospy.loginfo("finish!")
            sys.exit()

if __name__ == "__main__":
    rospy.init_node("task_2_image_saver", anonymous=True)
    controller = image_saver()
    rospy.spin()
