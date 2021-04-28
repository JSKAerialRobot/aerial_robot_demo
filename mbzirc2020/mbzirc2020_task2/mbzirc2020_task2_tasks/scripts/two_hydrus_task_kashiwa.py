#!/usr/bin/env python

import math

import rospy

import tf2_ros
import tf.transformations as tft
import numpy as np
import ros_numpy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from std_srvs.srv import Empty as EmptyService
import std_srvs.srv

from task2_hydrus_interface import Task2HydrusInterface

class TaskStart:

    def __init__(self):
        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.TaskStartCallback)

        self.task_start = False

    def TaskStartCallback(self,msg):
        self.task_start = True

if __name__ == '__main__':

    rospy.init_node('two_hydrus_task')

    task_start = TaskStart()

    hydrus1 = Task2HydrusInterface(robot_ns='hydrus1')
    hydrus2 = Task2HydrusInterface(robot_ns='hydrus2')
    #sleep to wait for ros-clients' ready
    rospy.sleep(1)

    simulation = rospy.get_param('/simulation',default=False)

    global_place_channel_center_pos_gps = rospy.get_param('~global_place_channel_center_pos_gps')
    global_place_channel_yaw = rospy.get_param('~global_place_channel_yaw')
    place_channel_length = rospy.get_param('~place_channel_length')
    grasping_yaw = rospy.get_param('~grasping_yaw')
    recognition_wait = rospy.get_param('~recognition_wait')
    channel_pos_thresh = rospy.get_param('~channel_pos_thresh')
    rospy.loginfo(simulation)
    rospy.loginfo(global_place_channel_center_pos_gps)
    rospy.loginfo(recognition_wait)
    rospy.loginfo(channel_pos_thresh)

    while not task_start.task_start:
        rospy.sleep(1.0)

    #enable alt sensor, disable plane detection
    if not simulation:
        rospy.loginfo('enable alt sensor, disable plane detection')
        hydrus1.enable_alt_sensor(flag = True)
        hydrus2.enable_alt_sensor(flag = True)

        hydrus1.enable_plane_detection(flag = False)
        hydrus2.enable_plane_detection(flag = False)
        rospy.loginfo('complete: enable alt sensor, disable plane detection')
        rospy.sleep(1)

    #start
    hydrus1.start()
    hydrus2.start()
    rospy.sleep(1)

    #takeoff
    hydrus1.takeoff()
    hydrus2.takeoff()
    rospy.sleep(1)

    #wait for hovering
    rospy.loginfo("wait for hovering")
    hydrus1.wait_for_hovering()
    hydrus2.wait_for_hovering()
    rospy.loginfo("complete: wait for hovering")
    rospy.sleep(1)

    # change height of both robots
    hydrus1.goPos('global', None, 2.0, None)
    hydrus2.goPos('global', None, 2.0, None)

    #enable plane detection, disable alt sensor
    if not simulation:
        rospy.loginfo('enable plane detection, disable alt sensor')
        hydrus1.enable_plane_detection(flag = True)
        hydrus2.enable_plane_detection(flag = True)

        hydrus1.enable_alt_sensor(flag = False)
        hydrus2.enable_alt_sensor(flag = False)
        rospy.loginfo('complete: enable plane detection, disable alt sensor')
        rospy.sleep(1)

    # change height of both robots
    hydrus1.goPos('global', None, 3.5, None)
    hydrus2.goPos('global', None, 3.5, None)

    #change the follower machine to vel-nav mode
    rospy.loginfo("change the follower machine to vel-nav mode")
    hydrus2.change_ctrl_mode(mode='vel')
    rospy.loginfo("complete: change the follower machine to vel-nav mode")
    rospy.sleep(1)

    #set robots to yaw_free_ctrl
    rospy.loginfo("set robots to yaw_free_ctrl")
    hydrus1.set_yaw_free(flag=True)
    hydrus2.set_yaw_free(flag=True)
    rospy.loginfo("complete: set robots to yaw_free_ctrl")
    rospy.sleep(1)

    #enable plane detection, disable alt sensor
    if not simulation:
        rospy.loginfo('enable plane detection, disable alt sensor')
        hydrus1.enable_plane_detection(flag = True)
        hydrus2.enable_plane_detection(flag = True)

        hydrus1.enable_alt_sensor(flag = False)
        hydrus2.enable_alt_sensor(flag = False)
        rospy.loginfo('complete: enable plane detection, disable alt sensor')
        rospy.sleep(1)

    #the leader machine approaches the channel center with gps value
    rospy.loginfo('the leader machine approaches the channel center with gps value')

    hydrus1.goPosWaitConvergence(
        'global',
        global_place_channel_center_pos_gps, 
        None, 
        None, 
        gps_mode = True, 
        timeout=20, 
        pos_conv_thresh = 0.3, 
        yaw_conv_thresh = 1.0, 
        vel_conv_thresh = 0.1)
    rospy.loginfo('complete: the leader machine approaches the channel center with gps value')
    rospy.sleep(1)

    # the leader machine approaches the place position with channel info
    rospy.loginfo('the leader machine approaches the place position with channel info ')

    baselink_pos = hydrus1.getBaselinkPos()[0:2]
    rospy.loginfo(baselink_pos)
    rospy.loginfo(global_place_channel_yaw)
    target_pos = [baselink_pos[0]+(place_channel_length/2)*np.cos(global_place_channel_yaw),
                  baselink_pos[1]+(place_channel_length/2)*np.sin(global_place_channel_yaw)]
    rospy.loginfo(target_pos)
    hydrus1.goPosWaitConvergence(
        'global', 
        target_pos, 
        None, 
        None, 
        timeout=10, 
        pos_conv_thresh = 0.3, 
        yaw_conv_thresh = 0.1, 
        vel_conv_thresh = 0.1)
    rospy.loginfo('complete: the leader machine approaches the place position with channel info ')
    rospy.sleep(1)


    # the leader machine adjusts the place position with recognition
    rospy.loginfo('the leader machine adjusts the place position with recognition')
    try:
        channel_trans = hydrus1.getTF('hydrus1/channel_center', wait=recognition_wait)
        channel_trans = ros_numpy.numpify(channel_trans.transform)
        channel_pos = tft.translation_from_matrix(channel_trans)
        rospy.loginfo("succeed to find channel x: %f, y: %f", channel_pos[0], channel_pos[1])
        channel_pos_local = channel_pos - hydrus1.getBaselinkPos()

        hydrus1.goPosWaitConvergence(
            'global', 
            channel_pos[0:2], 
            None, 
            None, 
            pos_conv_thresh = 0.25, 
            yaw_conv_thresh = 0.1, 
            vel_conv_thresh = 0.1, 
            timeout = 10)

        # check channel is directly below
        if np.linalg.norm(channel_pos_local[0:2]) > channel_pos_thresh:
            rospy.logwarn("succeed to find channel, but not directly below. diff: %f", np.linalg.norm(channel_pos_local[0:2]))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("channel position detect failed")
    rospy.loginfo('complete: the leader machine adjusts the place position with recognition')
    rospy.sleep(1)

    #change the leader machine to pos-nav mode
    rospy.loginfo("change the leader machine to pos-nav mode")
    hydrus1.change_ctrl_mode(mode='pos')
    rospy.loginfo("complete: change the leader machine to pos-nav mode")
    rospy.sleep(1)

    #the follower machine approaches the place position
    rospy.loginfo("the follower machine approaches the place position")
    baselink_pos = hydrus2.getBaselinkPos()[0:2]
    baselink_yaw = hydrus2.getBaselinkRPY()[2]
    object_yaw = baselink_yaw + math.pi*(3.0/4)
    dist_between_machines = 2 
    rospy.loginfo(baselink_pos)
    rospy.loginfo(object_yaw)
    rospy.loginfo(global_place_channel_yaw)
    rospy.loginfo(np.cos(object_yaw))
    rospy.loginfo(np.cos(global_place_channel_yaw))
    rospy.loginfo(np.sin(object_yaw))
    rospy.loginfo(np.sin(global_place_channel_yaw))
    target_pos = [baselink_pos[0]+dist_between_machines*(np.cos(object_yaw)-np.cos(global_place_channel_yaw)),
                  baselink_pos[1]+dist_between_machines*(np.sin(object_yaw)-np.sin(global_place_channel_yaw))]
    rospy.loginfo(target_pos)
    hydrus2.goPosWaitConvergence(
        'global', 
        target_pos, 
        None, 
        None, 
        timeout=10, 
        pos_conv_thresh = 0.3, 
        yaw_conv_thresh = 0.1, 
        vel_conv_thresh = 0.1)
    rospy.loginfo("complete: the follower machine approaches the place position")
    rospy.sleep(1)

    # the follower machine adjusts the place position with recognition
    rospy.loginfo('the follower machine adjusts the place position with recognition')
    try:
        channel_trans = hydrus2.getTF('hydrus2/channel_center', wait=recognition_wait)
        channel_trans = ros_numpy.numpify(channel_trans.transform)
        channel_pos = tft.translation_from_matrix(channel_trans)
        rospy.loginfo("succeed to find channel x: %f, y: %f", channel_pos[0], channel_pos[1])
        channel_pos_local = channel_pos - hydrus2.getBaselinkPos()

        hydrus2.goPosWaitConvergence(
            'global', 
            channel_pos[0:2], 
            None, 
            None, 
            pos_conv_thresh = 0.25, 
            yaw_conv_thresh = 0.1, 
            vel_conv_thresh = 0.1, 
            timeout = 10)

        # check channel is directly below
        if np.linalg.norm(channel_pos_local[0:2]) > channel_pos_thresh:
            rospy.logwarn("succeed to find channel, but not directly below. diff: %f", np.linalg.norm(channel_pos_local[0:2]))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("channel position detect failed")
    rospy.loginfo('complete: the leader machine adjusts the place position with recognition')
    rospy.sleep(1)

    # change height of both robots
    hydrus1.goPos('global', None, 2.3, None)
    hydrus2.goPos('global', None, 2.3, None)
    rospy.sleep(5)

    #ungrasp
    rospy.loginfo("ungrasp")
    hydrus1.ungrasp(time=500)
    hydrus2.ungrasp(time=500)
    rospy.loginfo("complete: ungrasp")
    rospy.sleep(0.5)

    #set robots  yaw_ctrl
    rospy.loginfo("set robots yaw_ctrl")
    hydrus1.set_yaw_free(flag=False)
    hydrus2.set_yaw_free(flag=False)
    rospy.loginfo("complete set robots yaw_ctrl")
    rospy.sleep(5)

    # back to start position
    rospy.loginfo("back to start position")
    hydrus2.goPosWaitConvergence(
        'global', 
        [0.00,0.00], 
        None, 
        None, 
        timeout=10, 
        pos_conv_thresh = 0.3, 
        yaw_conv_thresh = 0.1, 
        vel_conv_thresh = 0.1)
    rospy.sleep(5)
    hydrus1.goPosWaitConvergence(
        'global', 
        [0.00,0.00], 
        None, 
        None, 
        timeout=10, 
        pos_conv_thresh = 0.3, 
        yaw_conv_thresh = 0.1, 
        vel_conv_thresh = 0.1)
    rospy.loginfo("complete: back to start position")

    #land
    rospy.loginfo("land")
    hydrus1.land()
    hydrus2.land()
    rospy.loginfo("complete: land")
    rospy.sleep(1)

    rospy.loginfo("complete: all_tasks")
