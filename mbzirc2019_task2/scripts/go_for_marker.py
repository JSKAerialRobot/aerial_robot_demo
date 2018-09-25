#!/usr/bin/env python
import rospy
#import sys
import time
import math
import tf2_geometry_msgs
import tf
import tf2_ros

import numpy as np

from std_msgs.msg import Empty, String, Bool
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, JointState
from spinal.msg import DesireCoord
from hydrus.srv import AddExtraModule
from visualization_msgs.msg import Marker
from spinal.msg import FourAxisCommand

class HydrusGoForMarker:
    def init(self):
	
    	#make new node
        rospy.init_node("hydrus_go_for_marker", anonymous=True)

        ##current value
        self.uav_global_xy_pos_ = np.zeros(2)

	##target value
        self.target_xy_pos_ = np.zeros(2)
        self.target_object_pos_ = np.zeros(3)

        ##initial value
        self.initial_uav_global_xy_pos_ = np.zeros(2)
        self.uav_raw = 0

        ##about thrust
        self.thrust = {
	'free':(0, 0, 0, 0),
        'left':(10, 7, 0, 0),
        'right':(0, 0, 7, 10),
        'forward':(0, 7.5, 9.5, 0),
	}
        self.adhesion = 1
        self.count = 0
        self.period = 20
        self.end = 100

        #state machine
        self.INITIAL_STATE_ = 0
        self.SEARCH_STATE_ = 1
        self.GO_FOR_MARKER_STATE_ = 2
        self.GO_FOR_DESTINATION_STATE_ = 3
        self.END_STATE_ = 4
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "searching", "go_for_marker", "go_for_destination", "end"]
        self.task_start_ = False
        self.detect_object_ = False
        self.old_hantei = [-1,0,0]

        self.delay = 0

        #ros param
        ##setting control rate
	self.control_rate_ = rospy.get_param("~control_rate", 20)
        ##setting convergence threshold
	self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.65)
	##setting topics' name
        self.task_start_sub_topic_name_ = rospy.get_param("~task_start_sub_topic_name", "/task_start")
        self.uav_start_pub_topic_name_ = rospy.get_param("~uav_start_pub_topic_name_", "/teleop_command/start")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")
        self.uav_nav_pub_topic_name_ = rospy.get_param("~uav_nav_pub_topic_name", "/uav/nav")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "/state_machine")
        self.object_pub_topic_name_ = rospy.get_param("~object_pub_topic_name_", "/object")
        ##setting several parameter
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 1.0)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 1.0)

        ##setting srv name
        self.add_extra_module_srv_name_ = rospy.get_param("~add_extra_module_srv_name", "/add_extra_module")

        #subscriber
        self.task_start_sub_ = rospy.Subscriber(self.task_start_sub_topic_name_, Empty, self.taskStartCallback)
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)

        #publisher
        self.uav_start_pub_ = rospy.Publisher(self.uav_start_pub_topic_name_, Empty, queue_size = 10)
        self.uav_nav_pub_ = rospy.Publisher(self.uav_nav_pub_topic_name_, FlightNav, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)

        self.object_pub = rospy.Publisher(self.object_pub_topic_name_, Marker, queue_size=10)
        self.ctrl_four_axis_pub = rospy.Publisher('/aerial_robot_control_four_axis', FourAxisCommand, queue_size = 10)

        #tf
        ##listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.listener = tf.TransformListener()
        #service client
        self.add_extra_module_srv_client_ = rospy.ServiceProxy(self.add_extra_module_srv_name_, AddExtraModule)

        #timer
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)


    def taskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        self.task_start_ = True
        self.task_start_sub_.unregister()

    def uavOdomCallback(self, msg):
        self.uav_global_xy_pos_ = np.array([msg.pose.pose.position.x,
                                            msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_roll_ = rpy[0]
        self.uav_pitch_ = rpy[1]
        self.uav_yaw_ = rpy[2]
        self.uav_linear_vel_ = np.array([msg.twist.twist.linear.x,
                                         msg.twist.twist.linear.y,
                                         msg.twist.twist.linear.z])
        self.uav_q_ = q

    def isConvergent(self, target_xy_pos):
        rospy.loginfo ("target_x:%f uav_x:%f target_y:%f uav_y:%f,",target_xy_pos[0],self.uav_global_xy_pos_[0], target_xy_pos[1],self.uav_global_xy_pos_[1])
        delta_pos = np.array([target_xy_pos[0] - self.uav_global_xy_pos_[0], target_xy_pos[1] - self.uav_global_xy_pos_[1]])
        rospy.loginfo("delta_pos : %f", np.linalg.norm(delta_pos))
        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_:
            return True
        else:
            return False

    def go_for_destination(self, hantei, free=0):
        if hantei[0] < -0.35:
	    command = 'left'
        elif hantei[0] > 0.35:
	    command = 'right'
        else:
	    command = 'forward'
            self.count = 0
	if free == 1:
	    command = 'free'
	spinal = FourAxisCommand()
        spinal.angles = [-0.18, 0, 0]
        if hantei[1] >= 0:
            self.adhesion = 1 #- hantei[1] * 0.2
            self.thrust['left'] = (self.adhesion*10, self.adhesion*7, 0, 0)
	    self.thrust['right'] =(0, 0, self.adhesion*7, self.adhesion*10)
        #if hantei[2] <= 2:
         #   self.thrust['forward'] = (0, 5+hantei[2], 6+hantei[2], 0)
        spinal.base_throttle = self.thrust[command]
        #rospy.loginfo(self.thrust[command])
        self.ctrl_four_axis_pub.publish(spinal)

    def hantei(self, xy_pos):
        a = self.target_xy_pos_ - xy_pos
        b = np.array([np.cos(self.uav_yaw_-np.pi/4),np.sin(self.uav_yaw_-np.pi/4)])
        return (a[0]*b[1]-a[1]*b[0])/ (np.linalg.norm(a) * (np.linalg.norm(b))), np.dot(a,b)/ (np.linalg.norm(a) * (np.linalg.norm(b))), np.linalg.norm(a)

    def object_info_update(self, x, y, name, frame):
        marker_data = marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.time.now()

        marker_data.ns = "basic_shapes"
        marker_data.id = 0

        marker_data.action = marker.add

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.1

        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 1.0
        marker_data.pose.orientation.w = 0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.15
        marker_data.scale.y = 0.15
        marker_data.scale.z = 0.2

        marker_data.lifetime = rospy.duration()

        marker_data.type = 3

        self.object_pub.publish(marker_data)


    def controlCallback(self, event):
        try:
           #get the postion of the destination
           w2obj_tf = self.tfBuffer.lookup_transform('world', 'cluster_decomposeroutput00', rospy.Time())
           w2cam_tf = self.tfBuffer.lookup_transform('world', 'camera_rgb_optical_frame', rospy.Time(0))
           w2obj_tf.transform.rotation = w2cam_tf.transform.rotation
           self.target_xy_pos_[0] = w2obj_tf.transform.translation.x
           self.target_xy_pos_[1] = w2obj_tf.transform.translation.y
           self.detect_object_ = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo_throttle(10,"no tf ")

        #setting duty
        if self.count >= self.period:
            self.go_for_destination([1,0,0],1) #free
            self.count += 1
            if self.count >= self.end:
                self.count = 0
            return
        self.count += 1

        #navigation
        if self.state_machine_ == self.SEARCH_STATE_:
            self.go_for_destination([-1,-1,0]) #turn left
        if self.state_machine_ == self.GO_FOR_DESTINATION_STATE_:
            self.go_for_destination(self.hantei(self.uav_global_xy_pos_))
        if self.state_machine_ == self.END_STATE_:
            self.go_for_destination([1,0,0], 1) #free

        #rospy.loginfo(self.state_name_[self.state_machine_])

        #wait until task_start trigger
        if not self.task_start_:
            return

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.initial_uav_global_xy_pos_ = self.uav_global_xy_pos_
            self.uav_start_pub_.publish()
            time.sleep(1)
            self.old_hantei = self.hantei(self.uav_global_xy_pos_)
            self.state_machine_ = self.SEARCH_STATE_
            rospy.loginfo("init done")

        elif self.state_machine_ == self.SEARCH_STATE_:
            if self.detect_object_:
                self.state_machine_ = self.GO_FOR_DESTINATION_STATE_
                rospy.loginfo("detect target")

        elif self.state_machine_ == self.GO_FOR_DESTINATION_STATE_:
            self.period = 5
            self.end = 10
            if self.isConvergent(self.target_xy_pos_):
                self.state_machine_ = self.END_STATE_
                rospy.loginfo("reach target")

        elif self.state_machine_ == self.END_STATE_:
            self.delay = self.delay + 1
            if self.delay == self.control_rate_:
                rospy.loginfo("end")

        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

if __name__ == '__main__':
    try:
        hydrus_go_for_marker = HydrusGoForMarker()
        hydrus_go_for_marker.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
