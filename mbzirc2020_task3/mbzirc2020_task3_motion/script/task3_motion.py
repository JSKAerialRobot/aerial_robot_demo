#!/usr/bin/env python

""" search for target tf to be detected and call reaching motion """

import sys
import time
import math
from enum import Enum
import rospy
import tf2_ros

from subprocess import *
import numpy as np
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from hydrus_commander import HydrusCommander

class SearchPlannerState(Enum):
    STARTING = 0
    SEARCHING = 1
    FINISHED = 2
    FAILURE = 99

class SearchPlanner:
    """ search for target tf in designated square area """
    def __init__(self, target_tf_name='target_object', rate=1.0, area_corners=[[0,0],[10,10]], search_height=3.0, search_grid_size=1.0, reach_margin=0.2):
        self.target_tf_name = rospy.get_param('~target_tf_name', target_tf_name)
        self.rate = rospy.get_param('~search_rate', rate)
        area_corners = rospy.get_param('~area_corners', area_corners)
        self.start_pos = area_corners[0]
        self.goal_pos = area_corners[1]
        self.search_grid_size = rospy.get_param('~search_grid_size', search_grid_size)
        self.reach_margin = rospy.get_param('~reach_margin', reach_margin)
        self.search_height = rospy.get_param('~search_height', reach_margin)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.commander = HydrusCommander()

        self.state = SearchPlannerState.STARTING
        self.searched_grid = np.zeros((int((area_corners[1][0]-area_corners[0][0])/self.search_grid_size), int((area_corners[1][1]-area_corners[0][1])/self.search_grid_size)), dtype=bool)
        self.current_grid_idx = [0,0]
        self.x_move_dir = 1
        self.y_move_dir = 0

    def grid_pos(self, grid_idx):
        """return position from grid index"""
        # TODO check valid range
        return (self.start_pos[0]+self.search_grid_size*grid_idx[0],
                self.start_pos[1]+self.search_grid_size*grid_idx[1])

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

    def execute(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.state == SearchPlannerState.STARTING:
                rospy.loginfo("starting search planner")
                self.commander.change_height(self.search_height)
                init_pos = self.grid_pos(self.current_grid_idx)
                self.commander.move_to(init_pos[0], init_pos[1])
                self.state  = SearchPlannerState.SEARCHING
            elif self.state == SearchPlannerState.SEARCHING:
                try:
                    cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
                except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                    rospy.logwarn("tf lookup exception catched: could not find cog")
                    continue
                search_pos = self.grid_pos(self.current_grid_idx)
                self.commander.move_to(search_pos[0], search_pos[1])
                if abs(cog_pos.transform.translation.x - search_pos[0]) < self.reach_margin and abs(cog_pos.transform.translation.y - search_pos[1]) < self.reach_margin:
                    try:
                        target_pos = self.tfBuffer.lookup_transform('world', self.target_tf_name, rospy.Time(), rospy.Duration(0.5))
                        self.state = SearchPlannerState.FINISHED
                    except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                        rospy.logwarn("tf lookup exception catched: could not find target object")
                        if self.is_grid_full():
                            self.state = SearchPlannerState.FAILURE
                        else:
                            self.searched_grid[self.current_grid_idx[0],self.current_grid_idx[1]] = True
                            self.move_to_next_idx()
            elif self.state == SearchPlannerState.FINISHED:
                return self.state
            elif self.state == SearchPlannerState.FAILURE:
                return self.state
            else:
                self.state = SearchPlannerState.FAILURE
            rate.sleep()

class ReachPlannerState(Enum):
    APROACHING_ONTOP = 0
    DESCENDING = 1
    GRASPING = 2
    FINISHED = 3
    FAILURE = 99

class ReachPlanner:
    def __init__(self, target_tf_name='target_object', rate=1.0, approach_height=2.0, descending_height=1.0, approach_margin=1E-2, height_margin=5E-2, covering_pre_height=0.8, covering_post_height=0.1, covering_move_dist=1.0):
        """ reaching motion planner after target object was found """
        # TODO change into emach? state expression is not user friendly and destructive
        # constants for states
        self.target_tf_name = rospy.get_param('~target_tf_name', target_tf_name)
        self.rate = rospy.get_param('~reach_rate', rate)
        self.approach_height = rospy.get_param('~approach_height', approach_height)
        self.descending_height = rospy.get_param('~descending_height', descending_height)
        self.approach_margin = rospy.get_param('~approach_margin', approach_margin)
        self.height_margin = rospy.get_param('~height_margin', height_margin)
        self.covering_pre_height = rospy.get_param('~covering_pre_height', covering_pre_height)
        self.covering_post_height = rospy.get_param('~covering_post_height', covering_post_height)
        self.covering_move_dist = rospy.get_param('~covering_move_dist', covering_move_dist)

        # initial state
        self.state = ReachPlannerState.APROACHING_ONTOP

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.commander = HydrusCommander()

    def execute(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                if self.state == ReachPlannerState.APROACHING_ONTOP:
                    target_pos = self.tfBuffer.lookup_transform('world', self.target_tf_name, rospy.Time(), rospy.Duration(0.5))
                cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                rospy.logwarn("tf lookup exception catched")
                rate.sleep()
                continue

            if self.state == ReachPlannerState.APROACHING_ONTOP:
                self.commander.change_height(self.approach_height)
                self.commander.move_to(target_pos.transform.translation.x, target_pos.transform.translation.y)
                if abs(target_pos.transform.translation.x - cog_pos.transform.translation.x) < self.approach_margin and abs(target_pos.transform.translation.y - cog_pos.transform.translation.y) < self.approach_margin:
                    self.state = ReachPlannerState.DESCENDING
            elif self.state == ReachPlannerState.DESCENDING:
                self.commander.change_height(self.descending_height)
                if abs(cog_pos.transform.translation.z - self.descending_height) < self.height_margin:
                    self.state = ReachPlannerState.GRASPING
            elif self.state == ReachPlannerState.GRASPING:
                self.commander.covering_motion(cog_pos.transform.translation.x, cog_pos.transform.translation.y, self.covering_pre_height, self.covering_post_height, self.covering_move_dist)
                self.state = ReachPlannerState.FINISHED
                pass
            elif self.state == ReachPlannerState.FINISHED:
                return self.state
            elif self.state == ReachPlannerState.FAILURE:
                return self.state
            else:
                return ReachPlannerState.FAILURE
            rate.sleep()

def main():
    rospy.init_node('task3_motion_planner')
    commander = HydrusCommander()
    commander.close_joints()

    searcher = SearchPlanner()
    result = searcher.execute()
    # TODO Check result
    if result == SearchPlannerState.FINISHED:
        rospy.loginfo("Search Planner successfully finished, starting Reach Planner")
    elif result == SearchPlannerState.FAILURE:
        rospy.logerr("Search Planner FAILED")
        return
    else:
        rospy.logerr("Search Planner UNKNOWN")
        return

    planner = ReachPlanner();    
    result = planner.execute()
    # TODO Check result
    if result == ReachPlannerState.FINISHED:
        rospy.loginfo("Reach Planner successfully finished")
    elif result == ReachPlannerState.FAILURE:
        rospy.logerr("Reach Planner FAILED")
    else:
        rospy.logerr("Reach Planner UNKNOWN")

    return

if __name__ == "__main__":
    main()
