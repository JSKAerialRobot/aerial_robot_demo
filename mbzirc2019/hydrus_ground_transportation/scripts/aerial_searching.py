#!/usr/bin/env python
import rospy
import time
import math
import tf2_geometry_msgs
import tf
import tf2_ros

import numpy as np

from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from visualization_msgs.msg import Marker

class AerialSearch:
    def init(self):

        #make new node
        rospy.init_node("hydrus_object_recognition", anonymous=True)

        #initalize variable
        self.delay = 0
        ##current value
        self.uav_z_vel = 0
        self.uav_global_xy_pos_ = np.zeros(2)

        ##target value
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_z_vel_ = 0.05

        ##initial value
        self.initial_uav_global_xy_pos_ = np.zeros(2)
        self.initial_uav_yaw_ = 0.0

        self.target_joint_ = [[1.0, 1.2,  1.0],
                              [  1.5,  1.7, 1.5]]

        self.joint_name_ = ["joint1", "joint2", "joint3"]
        self.searching_point_index_ = 0

        ##about object
        self.detect_object_ = False
        self.target_object_pos_ = np.zeros(3)
        self.target_object_global_xy_pos_ = np.zeros(2)

        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.OBJECT_SEARCHING_STATE_ = 2
        self.OBJECT_APPROACHING_STATE_ = 3
        self.TRANSFORM_STATE_ = 4
        self.LANDING_STATE_ = 5
        self.GRASPING_STATE_ = 6
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "object_searching", "object_approaching", "transformation", "landing", "grasping"]
        self.task_start_ = False

        #ros param
        ##setting control rate
        self.control_rate_ = rospy.get_param("~control_rate", 20.0)

        ##setting search parameter
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.4)
        self.nav_vel_convergence_thresh_ = rospy.get_param("nav_vel_convergence_thresh_", 0.02)

        ##setting convergence threshold
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 0.5)
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 1.0)

        ##setting topics' name
        self.task_start_sub_topic_name_ = rospy.get_param("~task_start_sub_topic_name", "/task_start")
        self.uav_start_pub_topic_name_ = rospy.get_param("~uav_start_pub_topic_name_", "/teleop_command/start")
        self.uav_takeoff_pub_topic_name_ = rospy.get_param("~uav_takeoff_pub_topic_name", "/teleop_command/takeoff")
        self.uav_land_pub_topic_name_ = rospy.get_param("~uav_land_pub_topic_name", "/teleop_command/land")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/cog/odom")
        self.uav_nav_pub_topic_name_ = rospy.get_param("~uav_nav_pub_topic_name", "/uav/nav")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "/state_machine")
        self.joint_control_pub_topic_name_ = rospy.get_param("~joint_control_pub_topic_name", "/hydrusx/joints_ctrl")
        self.object_pub_topic_name_ = rospy.get_param("~object_pub_topic_name_", "/object")

        #subscriber
        self.task_start_sub_ = rospy.Subscriber(self.task_start_sub_topic_name_, Empty, self.taskStartCallback)
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)

        #publisher
        self.uav_start_pub_ = rospy.Publisher(self.uav_start_pub_topic_name_, Empty, queue_size = 10)
        self.uav_takeoff_pub_ = rospy.Publisher(self.uav_takeoff_pub_topic_name_, Empty, queue_size = 10)
        self.uav_land_pub_ = rospy.Publisher(self.uav_land_pub_topic_name_, Empty, queue_size = 10)
        self.uav_nav_pub_ = rospy.Publisher(self.uav_nav_pub_topic_name_, FlightNav, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.joint_control_pub_ = rospy.Publisher(self.joint_control_pub_topic_name_, JointState, queue_size = 10)
        self.object_pub = rospy.Publisher(self.object_pub_topic_name_, Marker, queue_size=10)

        #timer
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        #tf
        ##listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def taskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        self.task_start_ = True
        self.task_start_sub_.unregister()

    def uavOdomCallback(self, msg):
        self.uav_global_xy_pos_ = np.array([msg.pose.pose.position.x,
                                            msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        uav_q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.uav_yaw_ = tf.transformations.euler_from_quaternion(uav_q)[2]
        self.uav_linear_vel_ = np.array([msg.twist.twist.linear.x,
                                         msg.twist.twist.linear.y,
                                         msg.twist.twist.linear.z])

    def isConvergent(self):
        delta_pos = np.array([self.target_xy_pos_[0] - self.uav_global_xy_pos_[0], self.target_xy_pos_[1] - self.uav_global_xy_pos_[1], self.target_z_pos_ - self.uav_z_pos_])

        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and np.linalg.norm(self.uav_linear_vel_) < self.nav_vel_convergence_thresh_:
            return True
        else:
            return False

    def goPos(self, target_xy_pos, target_z_pos):
        msg = FlightNav()
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.POS_MODE

        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG

        msg.target_pos_x = target_xy_pos[0]
        msg.target_pos_y = target_xy_pos[1]
        msg.target_pos_z = target_z_pos

        self.target_xy_pos_ = target_xy_pos
        self.target_z_pos_ = target_z_pos

        self.uav_nav_pub_.publish(msg)

    def setJoint(self, joint_index):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_name_
        joint_state.position = self.target_joint_[joint_index]
        self.joint_control_pub_.publish(joint_state)

    def object_info_update(self, x, y):
        marker_data = Marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.type = Marker.CYLINDER
        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.1
        marker_data.pose.orientation.w = 1.0
        marker_data.color.r = 1.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.15
        marker_data.scale.y = 0.15
        marker_data.scale.z = 0.2
        self.object_pub.publish(marker_data)

    def controlCallback(self, event):
        if not self.task_start_:
            return

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.initial_uav_global_xy_pos_ = self.uav_global_xy_pos_
            self.initial_uav_yaw_ = self.uav_yaw_
            self.uav_start_pub_.publish()
            time.sleep(0.5)
            self.uav_takeoff_pub_.publish()
            rospy.loginfo("take off")
            self.state_machine_ = self.TAKEOFF_STATE_
            self.goPos(self.initial_uav_global_xy_pos_, self.takeoff_height_)

        elif self.state_machine_ == self.TAKEOFF_STATE_:
            if self.isConvergent():
                self.state_machine_ = self.OBJECT_SEARCHING_STATE_
                rospy.loginfo("takeoff phase -> searching phase")

        elif self.state_machine_ == self.OBJECT_SEARCHING_STATE_:
            rospy.loginfo_throttle (0.5, "searching target object")
            self.goPos(self.initial_uav_global_xy_pos_, self.target_z_pos_ + self.target_z_vel_ / self.control_rate_)

            try:
                w2obj_tf = self.tfBuffer.lookup_transform('world', 'target_object', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #rospy.loginfo_throttle(0.5, "no target object detected")
                return

            self.delay += 1
            if self.delay == self.control_rate_: # 1 [sec]
                self.target_object_global_xy_pos_[0] = w2obj_tf.transform.translation.x
                self.target_object_global_xy_pos_[1] = w2obj_tf.transform.translation.y
                rospy.loginfo("found object, go to position: [%f, %f, %f]",
                              self.target_object_global_xy_pos_[0],
                              self.target_object_global_xy_pos_[1],
                              self.target_z_pos_)

                self.state_machine_ = self.OBJECT_APPROACHING_STATE_
                self.goPos(self.target_object_global_xy_pos_, self.target_z_pos_)
                self.delay = 0

        elif self.state_machine_ == self.OBJECT_APPROACHING_STATE_:
            rospy.loginfo_throttle (0.5, "move upon target ojbect")

            if self.isConvergent():
                self.state_machine_ = self.TRANSFORM_STATE_
                self.setJoint(0)

        elif self.state_machine_ == self.TRANSFORM_STATE_:
            rospy.loginfo_throttle (0.5, "transformation")
            self.delay += 1

            if self.isConvergent() and self.delay > self.control_rate_:
                try:
                    w2baselink_tf = self.tfBuffer.lookup_transform('world', 'fc', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logrr(0.5, "can not solve the transform from world to fc")
                    return

                self.target_object_global_xy_pos_[0] = w2baselink_tf.transform.translation.x
                self.target_object_global_xy_pos_[1] = w2baselink_tf.transform.translation.y
                self.goPos(self.target_object_global_xy_pos_, self.target_z_pos_)
                self.state_machine_ = self.LANDING_STATE_
                self.delay = 0

        elif self.state_machine_ == self.LANDING_STATE_:
            rospy.loginfo_throttle (0.5, "posiiton modification before landing")
            self.delay += 1

            if self.isConvergent() and self.delay > self.control_rate_:
                self.uav_land_pub_.publish()
                self.state_machine_ = self.GRASPING_STATE_
                self.delay = 0

        elif self.state_machine_ == self.GRASPING_STATE_:
            rospy.loginfo_throttle (0.5, "landing")
            self.delay += 1

            self.target_z_pos_ = self.uav_z_pos_
            self.target_object_global_xy_pos_ = self.uav_global_xy_pos_

            if self.isConvergent()  and self.delay > self.control_rate_:
                rospy.loginfo ("grasping")
                self.setJoint(1)
                self.task_start_ = False

        if self.state_machine_ > self.OBJECT_SEARCHING_STATE_:
            self.object_info_update(self.target_object_global_xy_pos_[0], self.target_object_global_xy_pos_[1])

        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

if __name__ == '__main__':
    try:
        hydrus_aerial_search = AerialSearch()
        hydrus_aerial_search.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
