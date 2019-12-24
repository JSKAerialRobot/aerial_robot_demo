#!/usr/bin/env python

""" search fire in tower """

import rospy
import smach
import smach_ros

from hydrus_commander import HydrusCommander

from plane_search_states import *
from hydrus_navigation_states import *

from std_msgs.msg import Empty

def monitor_cb(ud, msg):
    return False

def main():
    """ main execution """
    rospy.init_node("find_ground_fire_motion")

    sm = smach.StateMachine(outcomes=['DONE', 'FAIL'])
    with sm:
        smach.StateMachine.add('INITIAL_STATE', smach_ros.MonitorState("~task3_start", Empty, monitor_cb), transitions={'invalid':'TAKEOFF', 'valid':'INITIAL_STATE', 'preempted':'INITIAL_STATE'})
        smach.StateMachine.add('TAKEOFF', TakeoffState(), transitions={'success':'NAVIGATING', 'fail':'INITIAL_STATE'})

        waypoint_nav_creator = WaypointNavigationStateMachineCreator()
        waypoints = rospy.get_param('~initial_waypoints', [[0,0,1]])
        sm_sub_nav = waypoint_nav_creator.create(waypoints)
        smach.StateMachine.add('NAVIGATING', sm_sub_nav, transitions={'success':'SEARCHING', 'failure':'FAIL'})

        search_sm_creator = PlaneFireFightStateMachineCreator()
        sm_sub = search_sm_creator.create('rectangular_grid')
        smach.StateMachine.add('SEARCHING', sm_sub, transitions={'success':'DONE', 'failure':'FAIL'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()

