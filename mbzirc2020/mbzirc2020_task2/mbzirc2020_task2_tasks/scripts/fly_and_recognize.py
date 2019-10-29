#!/usr/bin/env python
# from __future__ import print_function
import sys
import rospy
import time
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3Stamped
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
import tf
import tf2_ros
import numpy as np
from spinal.msg import FourAxisCommand
from copy import deepcopy
import math
import sys

from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import ModelCoefficientsArray

__author__ = "kuromiya@jsk.imi.i.u-tokyo.ac.jp (Naoki Kuromiya)"

class flying_controller():
    def __init__(self):

        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")


        #  publishers
        self.publish_start = rospy.Publisher("/teleop_command/start", Empty, queue_size=1)
        self.publihsh_takeoff = rospy.Publisher("/teleop_command/takeoff", Empty, queue_size=1)
        self.publish_flight = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.publish_joints_ctrl = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)
        self.publish_extra_joints_ctrl = rospy.Publisher("/hydrusx/extra_servos_ctrl", JointState, queue_size=1)
        self.publish_target_point = rospy.Publisher("/world_target_point", Point, queue_size=1)  #  target destination in world coordinate

        self.publish_working_fhase = rospy.Publisher("/working_fhase/py", Int32, queue_size=1)  #  working fhase of python node
        self.task_start = False
        self.task_number = 0

        #  subscribers
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        self.target_sub = rospy.Subscriber("/target_object/pos", Vector3Stamped, self.targetCallback, queue_size=1, buff_size=2)
        self.target_angle_sub = rospy.Subscriber("/target_object/angle", Float64, self.targetAngleCallback, queue_size=1, buff_size=2)
        self.working_fhase_sub = rospy.Subscriber("/working_fhase/cpp", Int32, self.workingfhaseCallback, queue_size=1, buff_size=2)

        #  message instances
        self.pose = PoseStamped()
        self.empty = Empty()
        self.flight = FlightNav()
        self.joints = JointState()

        self.WAIT = 0.5

        #additional
        self.capture_image = False
        self.count = 0

        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        #  target
        self.local_target_xy_pos_ = np.zeros(2)
        self.local_target_z_pos_ = 0.0
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_yaw_ = 0.0
        self.target_frame_ = self.GLOBAL_FRAME_
        self.detect_object_ = False

        self.uav_xyz_pos = np.zeros(3)
        self.move_to_target = False
        self.odom_update_flag_ = False
        self.uav_xy_global_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.uav_accumulated_yaw_ = 0.0
        self.camera_pose = [0.0]
        self.coef = []
        self.detect_count = 0

        self.working_fhase = 10
        self.task_start_time_ = rospy.Time.now()
        self.task_elapsed_time_ = rospy.Time(0)

        ##define finisher
        self.approached = False

        ##listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.listener = tf.TransformListener()

        #timer
        self.control_rate_ = rospy.get_param("~control_rate", 20)

        self.yaw_adjust = True

    def takeoff(self):
        # arm and takeoff
        time.sleep(self.WAIT)
        self.publish_start.publish(self.empty)
        time.sleep(self.WAIT)
        self.publihsh_takeoff.publish(self.empty)

    def goPos(self, x, y):
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
        msg.target_pos_z = 1.8
        time.sleep(self.WAIT)
        self.publish_flight.publish(msg)
        time.sleep(self.WAIT)

    def open_joints(self):
        """open links"""
        msg = JointState()
        msg.position = [1.40, 1.57, 1.40]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)
        msg.position = [1.25, 1.57, 1.25]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)
        msg.position = [1.0, 1.57, 1.0]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)


    def close_joints(self):
        """close links"""
        msg = JointState()

        while msg.effort < 1.0:
            msg.position = [msg.position[0] + 0.1, msg.position[1], msg.position[2] + 0.1]
            self.publish_joints_ctrl.publish(msg)
            time.sleep(3)

    def uavOdomCallback(self, msg):
        self.uav_xyz_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_yaw_ = rpy[2]

    def targetCallback(self, msg):
        new_msg = FlightNav()
        new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.pos_z_nav_mode = FlightNav.POS_MODE

        new_msg.header.stamp = rospy.Time.now()
        new_msg.control_frame = FlightNav.WORLD_FRAME
        new_msg.target = FlightNav.BASELINK
        new_msg.target_pos_x = 0.0
        new_msg.target_pos_y = 0.0
        new_msg.target_pos_z = 1.8
        new_msg.target_psi = 0.0

        target_arrive = 0

        if self.working_fhase == 0 and target_arrive == 0:
            target_x = msg.vector.x
            target_y = msg.vector.y
            target_z = 4.0  #  to watch from the sky

            print(target_y)

            if target_y > 3.0 and target_y < 20.0:
                new_msg.target_pos_x = target_x
                new_msg.target_pos_y = target_y
                new_msg.target_pos_z = target_z
                time.sleep(5)
                # self.publish_flight.publish(new_msg)
                time.sleep(3)
                time.sleep(70)
                target_arrive = 1
                camera_angle = JointState()
                camera_angle.position = [1.57]
                # self.publish_extra_joints_ctrl.publish(camera_angle)
                time.sleep(5)
                working_msg = Int32()
                working_msg.data = 0
                self.publish_working_fhase.publish(working_msg)

            else:
                print("reached !!")

        elif self.working_fhase == 3 and self.count == 0:
            working_msg = Int32()
            working_msg.data = 17
            target_angle = self.angle
            initial_angle = 0.0  #initial pose
            new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.psi_nav_mode = FlightNav.POS_MODE
            new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_psi = initial_angle
            # self.publish_flight.publish(new_msg)
            print("initial!")

            # sys.exit()

            self.open_joints()
            time.sleep(10)
            target_x = msg.vector.x
            target_y = msg.vector.y
            target_z = msg.vector.z + 1.0  #  to watch from the sky

            print("target_x : {}".format(target_x))
            print("target_y : {}".format(target_y))

            new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.psi_nav_mode = FlightNav.POS_MODE
            new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_psi = target_angle

            print(target_angle)
            self.publish_flight.publish(new_msg)
            print("rotate")
            time.sleep(20)
            
            new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.pos_z_nav_mode = FlightNav.POS_MODE
            
            
            new_msg.target_pos_x = target_x
            new_msg.target_pos_y = target_y
            new_msg.target_pos_z = target_z

            print(target_x)
            print(target_y)
            print(target_z)
            self.publish_flight.publish(new_msg)
            print("move!")
            self.count = 1
            time.sleep(10)

            target_z -= 0.5
            new_msg.target_pos_x = target_x
            new_msg.target_pos_y = target_y
            new_msg.target_pos_z = target_z
            self.publish_flight.publish(new_msg)
            print("move!")
            self.count = 1
            time.sleep(10)
            
            target_z -= 0.5
            new_msg.target_pos_x = target_x
            new_msg.target_pos_y = target_y
            print("target_angle : {}".format(target_angle))
            new_msg.target_pos_z = target_z
            self.publish_flight.publish(new_msg)
            print("move!")
            self.count = 1
            time.sleep(10)

            target_z -= 0.1
            new_msg.target_pos_x = target_x
            new_msg.target_pos_y = target_y
            new_msg.target_pos_z = target_z
            self.publish_flight.publish(new_msg)
            print("move!")
            self.count = 1
            time.sleep(10)

                
            self.close_joints()
            time.sleep(10)
            
    def targetAngleCallback(self, msg):
        self.angle = msg.data

    def workingfhaseCallback(self, msg):
        self.working_fhase = msg.data



if __name__ == "__main__":
    rospy.init_node("task_2_fly_and_recognize", anonymous=True)
    controller = flying_controller()
    rospy.spin()
