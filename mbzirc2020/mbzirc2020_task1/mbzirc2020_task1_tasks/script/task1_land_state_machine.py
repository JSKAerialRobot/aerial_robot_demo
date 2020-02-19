#!/usr/bin/env python

""" search fire in tower """

import rospy
import smach
import smach_ros

from hydrus_commander import HydrusCommander
from hydrus_navigation_states import *

from std_msgs.msg import Empty

def monitor_cb(ud, msg):
    return False

def main():
    """ main execution """
    rospy.init_node("task1_land_state_machine")

    sm = smach.StateMachine(outcomes=['DONE', 'FAIL'])
    with sm:
        nav_control_rate = rospy.get_param('~control_rate', 0.5)
        nav_approach_margin = rospy.get_param('~navigation_approach_margin', [0.5, 0.5, 0.5])

        nav_mode = rospy.get_param('~nav_mode', 2)

        smach.StateMachine.add('INITIAL_STATE', smach_ros.MonitorState("~task1_start", Empty, monitor_cb), transitions={'invalid':'TAKEOFF', 'valid':'INITIAL_STATE', 'preempted':'INITIAL_STATE'})
        smach.StateMachine.add('TAKEOFF', TakeoffState(15), transitions={'success':'NAVIGATING0', 'fail':'INITIAL_STATE'})

        waypoints_list = rospy.get_param('~initial_waypoints', [[[0,0,1]]])
        waypoints_number = len(waypoints_list)
        initial_yaw_flag = rospy.get_param('~initial_yaw_flag', False)
        initial_yaw = rospy.get_param('~initial_yaw', 0.0)
        # smach.StateMachine(outcomes=['NAVIGATING'+str(waypoints_number)])
        # debug
        print ("waypoint: ")
        print (waypoints_list)
        for i in range(waypoints_number):
            waypoint_nav_creator = GpsWaypointNavigationStateMachineCreator()
            waypoints = waypoints_list[i]
            sm_sub_nav = waypoint_nav_creator.create(waypoints, nav_approach_margin, nav_control_rate, nav_mode, initial_yaw_flag, initial_yaw)
            if i < (waypoints_number - 1):
                smach.StateMachine.add('NAVIGATING'+str(i), sm_sub_nav, transitions={'success':'NAVIGATING'+str(i+1), 'failure':'FAIL'})
            else:
                smach.StateMachine.add('NAVIGATING'+str(i), sm_sub_nav, transitions={'success':'TASK1_ENTER_LAND_STATE', 'failure':'FAIL'})
        # smach.StateMachine.add('TASK1_ENTER_LAND_STATE', PrepareTaskState(), transitions={'success':'TASK1_ENTER_LAND_STATE_PREPARED', 'fail':'FAIL'})

        # smach.StateMachine.add('TASK1_ENTER_LAND_STATE_PREPARED', LandingState(15), transitions={'success':'DONE', 'fail':'INITIAL_STATE'})

        # smach.StateMachine.add('TASK1_ENTER_LAND_STATE', LandingState(15), transitions={'success':'DONE', 'fail':'INITIAL_STATE'})
        smach.StateMachine.add('TASK1_ENTER_LAND_STATE', LandingStateWithOpenJoints(15), transitions={'success':'DONE', 'fail':'INITIAL_STATE'})
        

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()

