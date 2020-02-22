#!/usr/bin/env python

import rospy
import smach
import smach_ros

import tf
import tf2_ros
import numpy as np


from hydrus_commander import HydrusCommander
from task_commander import TaskCommander

class PrepareEstimation(smach.State):
    def __init__(self, wait_time=0.5):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.task_commander = TaskCommander()
        self.wait_time = wait_time

    def execute(self, userdata):
        rospy.sleep(self.wait_time)
        self.task_commander.prepare_estimation()
        return 'success'

class StopEstimation(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.task_commander = TaskCommander()

    def execute(self, userdata):
        self.task_commander.stop_estimation()
        return 'success'

class PrepareTaskState(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['first_waypoint', 'current_waypoint', 'fail'], input_keys=['first_waypoint_flag_input', 'return_first_waypoint_flag_input'], output_keys=['first_waypoint_flag_output'])
        self.task_commander = TaskCommander()
        self.hydrus_commander = HydrusCommander()

    def execute(self, userdata):
        ## 1. estimation stops
        ## 2. joints open to initial state
        self.task_commander.stop_estimation()
        self.hydrus_commander.open_joints()
        if userdata.first_waypoint_flag_input or userdata.return_first_waypoint_flag_input:
            userdata.first_waypoint_flag_output = False
            return 'first_waypoint'
        else:
            return 'current_waypoint'

class TakeoffState(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.commander = HydrusCommander()
        self.wait_time = wait_time

    def execute(self, userdata):
        self.commander.arm_and_takeoff()
        rospy.sleep(self.wait_time)
        return 'success'

class LandingState(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.commander = HydrusCommander()
        self.wait_time = wait_time

    def execute(self, userdata):
        self.commander.land()
        rospy.sleep(self.wait_time)
        return 'success'

class LandingStateWithOpenJoints(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.commander = HydrusCommander()

    def execute(self, userdata):
        self.commander.open_joints()
        rospy.sleep(9.0)
        self.commander.land()
        rospy.sleep(self.wait_time)
        return 'success'

class GoPositionState(smach.State):
    def __init__(self, target_pos=[0,0,0], approach_margin=[0.05, 0.05, 0.05], control_rate=5.0, nav_mode=2):
        smach.State.__init__(self, outcomes=['success', 'ongoing'])
        self.commander = HydrusCommander(nav_mode=nav_mode)

        self.control_rate = control_rate
        self.approach_margin = approach_margin

        self.target_pos = target_pos
        self.cmd_pub_flag = False

    def execute(self, userdata):
        if not self.cmd_pub_flag:
            print(self.target_pos)
            if len(self.target_pos) == 3:
                self.commander.move_to_xyz(self.target_pos[0], self.target_pos[1], self.target_pos[2])
            elif len(self.target_pos) == 1:
                self.commander.change_height(self.target_pos[0])
            self.cmd_pub_flag = True
            rospy.loginfo("Navigating to waypoint "+str(self.target_pos))

        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.approach_margin[0] and abs(target_pos_err[1]) < self.approach_margin[1] and abs(target_pos_err[2]) < self.approach_margin[2]:
            self.cmd_pub_flag = False
            return 'success'

        rospy.sleep(1/self.control_rate)
        return 'ongoing'

class GoGpsPositionState(smach.State):
    def __init__(self, target_pos=[0,0,0], approach_margin=[0.05, 0.05, 0.05], control_rate=5.0, nav_mode=5, initial_yaw_flag=False, initial_yaw=0.0):
        smach.State.__init__(self, outcomes=['success', 'ongoing'])
        self.commander = HydrusCommander(nav_mode=nav_mode)

        self.control_rate = control_rate
        self.approach_margin = approach_margin
        self.__initial_yaw_flag = initial_yaw_flag
        self.__initial_yaw = initial_yaw

        self.target_pos = target_pos
        self.cmd_pub_flag = False

    def execute(self, userdata):
        if not self.cmd_pub_flag:
            print(self.target_pos)
            if len(self.target_pos) == 3:
                if self.__initial_yaw_flag:
                    self.commander.move_to_xyz_yaw(self.target_pos[0], self.target_pos[1], self.target_pos[2], self.__initial_yaw, override_nav_mode=5)
                else:
                    self.commander.move_to_xyz(self.target_pos[0], self.target_pos[1], self.target_pos[2], override_nav_mode=5)
            elif len(self.target_pos) == 1:
                if self.__initial_yaw_flag:
                    self.commander.change_height_yaw(self.target_pos[0], self.__initial_yaw)
                else:
                    self.commander.change_height(self.target_pos[0])
            self.cmd_pub_flag = True
            rospy.loginfo("Navigating to waypoint "+str(self.target_pos))

        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.approach_margin[0] and abs(target_pos_err[1]) < self.approach_margin[1] and abs(target_pos_err[2]) < self.approach_margin[2]:
            self.cmd_pub_flag = False
            return 'success'

        rospy.sleep(1/self.control_rate)
        return 'ongoing'

class WaypointNavigationStateMachineCreator():
    def create(self, waypoints, approach_margin, control_rate, nav_mode=2):
        ''' waypoints: nx3 array of 3d poionts
        '''
        sm = smach.StateMachine(outcomes={'success', 'failure'})
        with sm:
            for i, waypoint in enumerate(waypoints):
                if i==len(waypoints)-1:
                    next_state = 'success'
                else:
                    next_state = 'waypoint'+str(i+1)
                smach.StateMachine.add('waypoint'+str(i), GoPositionState(waypoint, approach_margin, control_rate, nav_mode),
                    transitions={'success':next_state,
                                 'ongoing':'waypoint'+str(i)})
        return sm

class GpsWaypointNavigationStateMachineCreator():
    def create(self, waypoints, approach_margin, control_rate, nav_mode=5, initial_yaw_flag=False, initial_yaw=0.0):
        ''' waypoints: nx3 array of 3d poionts
        '''
        sm = smach.StateMachine(outcomes={'success', 'failure'})
        with sm:
            for i, waypoint in enumerate(waypoints):
                if i==len(waypoints)-1:
                    next_state = 'success'
                else:
                    next_state = 'waypoint'+str(i+1)
                rospy.sleep(0.1)
                smach.StateMachine.add('waypoint'+str(i), GoGpsPositionState(waypoint, approach_margin, control_rate, nav_mode, initial_yaw_flag, initial_yaw),
                    transitions={'success':next_state,
                                 'ongoing':'waypoint'+str(i)})
        return sm

