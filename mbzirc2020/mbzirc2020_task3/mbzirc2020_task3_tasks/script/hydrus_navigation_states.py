#!/usr/bin/env python

import rospy
import smach
import smach_ros

import tf
import tf2_ros
import numpy as np


from hydrus_commander import HydrusCommander

class TakeoffState(smach.State):
    def __init__(self, wait_time=20):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.commander = HydrusCommander()
        self.wait_time = wait_time

    def execute(self, userdata):
        self.commander.arm_and_takeoff()
        rospy.sleep(self.wait_time)
        return 'success'

class GoPositionState(smach.State):
    def __init__(self, target_pos=[0,0,0], approach_margin=[0.05, 0.05, 0.05], control_rate=5.0):
        smach.State.__init__(self, outcomes=['success', 'ongoing'])
        self.commander = HydrusCommander()

        self.control_rate = control_rate
        self.approach_margin = approach_margin
        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.target_pos = target_pos

    def execute(self, userdata):
        try:
            self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")
        self.commander.change_height(self.target_pos[2])
        self.commander.move_to(self.target_pos[0], self.target_pos[1])

        rospy.loginfo("Navigating to waypoint "+str(self.target_pos))

        if abs(self.target_pos[0] - self.cog_pos.transform.translation.x) < self.approach_margin[0] and abs(self.target_pos[1] - self.cog_pos.transform.translation.y) < self.approach_margin[1] and abs(self.target_pos[2] - self.cog_pos.transform.translation.z) < self.approach_margin[2]:
            return 'success'

        rospy.sleep(1/self.control_rate)
        return 'ongoing'

class WaypointNavigationStateMachineCreator():
    def create(self, waypoints, approach_margin, control_rate):
        ''' waypoints: nx3 array of 3d poionts
        '''
        sm = smach.StateMachine(outcomes={'success', 'failure'})
        with sm:
            for i, waypoint in enumerate(waypoints):
                if i==len(waypoints)-1:
                    next_state = 'success'
                else:
                    next_state = 'waypoint'+str(i+1)
                smach.StateMachine.add('waypoint'+str(i), GoPositionState(waypoint, approach_margin, control_rate),
                    transitions={'success':next_state,
                                 'ongoing':'waypoint'+str(i)})
        return sm


