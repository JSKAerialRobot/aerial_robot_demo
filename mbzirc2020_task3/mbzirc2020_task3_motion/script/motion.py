#!/usr/bin/env python

""" search for target tf to be detected and call reaching motion """

import sys
import time
import math
from enum import Enum
import rospy
import tf
import tf2_ros
import numpy as np

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3Stamped

from hydrus_commander import HydrusCommander

class Task3MotionState(Enum):
    INITIAL_STATE = 0
    SEARCHING = 1
    APROACHING_ONTOP = 2
    DESCENDING = 3
    COVERING = 4
    FINISHED = 5
    FAILURE = 99


class Task3Motion():
    def __init__(self):
        rospy.init_node('task3_motion', anonymous=True)

        # retrieve parameters
        self.target_topic_name = rospy.get_param('~target_topic_name', '/target_object/pos')
        self.control_rate = rospy.get_param('~control_rate', 5.0)
        self.area_corners = rospy.get_param('~area_corners', [[0,0],[10,10]])
        self.search_grid_size = rospy.get_param('~search_grid_size', 2.0)
        self.reach_margin = rospy.get_param('~reach_margin', 0.2)
        self.search_height = rospy.get_param('~search_height', 3.0)

        self.target_timeout = rospy.get_param('~target_timeout', 1.0)
        self.approach_height = rospy.get_param('~approach_height', 3.0)
        self.descending_height = rospy.get_param('~descending_height', 2.0)
        self.approach_margin = rospy.get_param('~approach_margin', 0.05)
        self.height_margin = rospy.get_param('~height_margin', 0.05)
        self.covering_pre_height = rospy.get_param('~covering_pre_height', 1.0)
        self.covering_post_height = rospy.get_param('~covering_post_height', 0.2)
        self.covering_move_dist = rospy.get_param('~covering_move_dist', 1.0)

        self.target_pos = None # set when target_pos topic recieved

        # set initial state
        self.state = Task3MotionState.INITIAL_STATE

        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # commander to send command to hydrus
        self.commander = HydrusCommander()

        # define subscriber
        self.task_start_sub_ = rospy.Subscriber("task3_start", Empty, self.task3StartCallback)
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)

        self.searched_grid = np.zeros((int((self.area_corners[1][0]-self.area_corners[0][0])/self.search_grid_size), int((self.area_corners[1][1]-self.area_corners[0][1])/self.search_grid_size)), dtype=bool)
        self.current_grid_idx = [0,0]
        self.x_move_dir = 1
        self.y_move_dir = 0

        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.controlCallBack)

    def grid_pos(self, grid_idx):
        """return position from grid index"""
        # TODO check valid range
        start_pos = self.area_corners[0]
        return (start_pos[0]+self.search_grid_size*grid_idx[0],
                start_pos[1]+self.search_grid_size*grid_idx[1])

    def is_grid_full(self):
        return np.all(self.searched_grid)

    def next_grid_idx_naive(self):
        result_idx = [0,0]
        if self.current_grid_idx[0] < self.searched_grid.shape[0]:
            result_idx[0] = self.current_grid_idx[0]+1
            result_idx[1] = self.current_grid_idx[1]
        elif self.current_grid_idx[1] < self.searched_grid.shape[1]:
            result_idx[0] = 0
            result_idx[1] = self.current_grid_idx[1] + 1
        return result_idx

    def move_to_next_idx(self):
        self.current_grid_idx[0] = self.current_grid_idx[0] + self.x_move_dir
        self.current_grid_idx[1] = self.current_grid_idx[1] + self.y_move_dir
        # TODO make it more efficient
        if self.current_grid_idx[0] == self.searched_grid.shape[0]-1:
            if self.y_move_dir == 0 and self.x_move_dir == 1:
                self.x_move_dir = 0
                self.y_move_dir = 1
            else:
                self.x_move_dir = -1
                self.y_move_dir = 0
        elif self.current_grid_idx[0] == 0:
            if self.y_move_dir == 0 and self.x_move_dir == -1:
                self.x_move_dir = 0
                self.y_move_dir = 1
            else:
                self.x_move_dir = 1
                self.y_move_dir = 0
        return

    def task3StartCallback(self, msg):
        """change state to start task 3"""
        rospy.loginfo("starting searching motion")
        self.commander.change_height(self.search_height)
        init_pos = self.grid_pos(self.current_grid_idx)
        self.commander.move_to(init_pos[0], init_pos[1])
        self.commander.close_joints()
        self.state = Task3MotionState.SEARCHING

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos = msg
        if self.state != Task3MotionState.INITIAL_STATE and self.state.value <= Task3MotionState.SEARCHING.value:
            self.state = Task3MotionState.APROACHING_ONTOP
        return

    def controlCallBack(self, event):
        """main state machine which invoked periodically by rospy.Timer"""
        # retrieve cog position from tf
        try:
            self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")

        # rospy.loginfo(self.state.value)
        # state machine
        if self.state == Task3MotionState.INITIAL_STATE:
            pass
        elif self.state == Task3MotionState.SEARCHING:
            search_pos = self.grid_pos(self.current_grid_idx)
            self.commander.change_height(self.search_height)
            self.commander.move_to(search_pos[0], search_pos[1])
            if abs(self.cog_pos.transform.translation.x - search_pos[0]) < self.reach_margin and abs(self.cog_pos.transform.translation.y - search_pos[1]) < self.reach_margin:
                if self.is_grid_full():
                    self.state = Task3MotionState.FAILURE
                else:
                    self.searched_grid[self.current_grid_idx[0],self.current_grid_idx[1]] = True
                    self.move_to_next_idx()
        elif self.state == Task3MotionState.APROACHING_ONTOP:
            if self.target_pos is None or rospy.Time.now() - self.target_pos.header.stamp > rospy.Duration(self.target_timeout) :  # TODO check timestamp of target to check if loosing sight
                rospy.logerr("lost target_object, returning to searching")
                self.target_pos = None
                self.state = Task3MotionState.SEARCHING
                return
            self.commander.change_height(self.approach_height)
            self.commander.move_to(self.target_pos.vector.x, self.target_pos.vector.y)
            if abs(self.target_pos.vector.x - self.cog_pos.transform.translation.x) < self.approach_margin and abs(self.target_pos.vector.y - self.cog_pos.transform.translation.y) < self.approach_margin:
                self.state = Task3MotionState.DESCENDING
        elif self.state == Task3MotionState.DESCENDING:
            self.commander.change_height(self.descending_height)
            if abs(self.cog_pos.transform.translation.z - self.descending_height) < self.height_margin:
                self.state = Task3MotionState.COVERING
        elif self.state == Task3MotionState.COVERING:
            rot = self.cog_pos.transform.rotation
            q = (rot.x, rot.y, rot.z, rot.w)
            cog_euler = tf.transformations.euler_from_quaternion(q)
            self.commander.covering_motion(self.cog_pos.transform.translation.x, self.cog_pos.transform.translation.y, cog_euler[2], self.covering_pre_height, self.covering_post_height, self.covering_move_dist)
            self.state = Task3MotionState.FINISHED
        elif self.state == Task3MotionState.FINISHED:
            rospy.loginfo("Task3 Motion successfully finished")
            self.state = Task3MotionState.INITIAL_STATE
        elif self.state == Task3MotionState.FAILURE:
            rospy.logerr("Task3 Motion FAILURE")
        else:
            self.state = Task3MotionState.FAILURE

        return

if __name__ == '__main__':
    try:
        task3_motion = Task3Motion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

