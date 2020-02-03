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

def track_flag_cb(ud, msg):
    return False

def return_initial_flag_cb(ud, msg):
    StopEstimation()
    return False

def main():
    """ main execution """
    rospy.init_node("task1_motion_state_machine")

    sm = smach.StateMachine(outcomes=['DONE', 'FAIL'])
    with sm:
        nav_control_rate = rospy.get_param('~control_rate', 1.0)
        nav_approach_margin = rospy.get_param('~navigation_approach_margin', [0.5, 0.5, 0.5])

        nav_mode = rospy.get_param('~nav_mode', 2)

        search_params = {}
        search_params['nav_mode'] = nav_mode
        search_params['target_topic_name'] = rospy.get_param('~target_topic_name', '/target_object/pos')
        search_params['control_rate'] = rospy.get_param('~control_rate', 5.0)
        search_params['search_grid_size'] = rospy.get_param('~search_grid_size', 1.0)
        search_params['reach_margin'] = rospy.get_param('~reach_margin', 0.2)
        search_params['search_height'] = rospy.get_param('~search_height', 3.0)
        search_params['target_timeout'] = rospy.get_param('~target_timeout', 1.0)
        search_params['approach_height'] = rospy.get_param('~approach_height', 3.0)
        search_params['descending_height'] = rospy.get_param('~descending_height', 2.0)
        search_params['approach_margin'] = rospy.get_param('~approach_margin', 0.05)
        search_params['height_margin'] = rospy.get_param('~height_margin', 0.05)
        search_params['covering_pre_height'] = rospy.get_param('~covering_pre_height', 1.0)
        search_params['covering_post_height'] = rospy.get_param('~covering_post_height', 0.2)
        search_params['covering_move_dist'] = rospy.get_param('~covering_move_dist', 1.0)

        smach.StateMachine.add('INITIAL_STATE', smach_ros.MonitorState("~task1_start", Empty, monitor_cb), transitions={'invalid':'TAKEOFF', 'valid':'INITIAL_STATE', 'preempted':'INITIAL_STATE'})
        smach.StateMachine.add('TAKEOFF', TakeoffState(), transitions={'success':'NAVIGATING0', 'fail':'INITIAL_STATE'})

        waypoints_list = rospy.get_param('~initial_waypoints', [[[0,0,1]]])
        waypoints_number = len(waypoints_list)
        # smach.StateMachine(outcomes=['NAVIGATING'+str(waypoints_number)])
        # debug
        print ("waypoint: ")
        print (waypoints_list)
        for i in range(waypoints_number):
            waypoint_nav_creator = GpsWaypointNavigationStateMachineCreator()
            waypoints = waypoints_list[i]
            sm_sub_nav = waypoint_nav_creator.create(waypoints, nav_approach_margin, nav_control_rate, nav_mode)
            if i < (waypoints_number - 1):
                smach.StateMachine.add('NAVIGATING'+str(i), sm_sub_nav, transitions={'success':'NAVIGATING'+str(i+1), 'failure':'FAIL'})
            else:
                smach.StateMachine.add('NAVIGATING'+str(i), sm_sub_nav, transitions={'success':'TASK1_ENTER_STATE', 'failure':'FAIL'})

        task1_waiting_waypoints_list = rospy.get_param('~task1_waiting_waypoint', [[[0,0,1]]])
        task1_waiting_waypoint_nav_creator = GpsWaypointNavigationStateMachineCreator()
        task1_waiting_waypoints = task1_waiting_waypoints_list[0]
        sm_sub_waiting_waypt_nav = task1_waiting_waypoint_nav_creator.create(task1_waiting_waypoints, nav_approach_margin, nav_control_rate, nav_mode)
        smach.StateMachine.add('TASK1_ENTER_STATE', sm_sub_waiting_waypt_nav, transitions={'success':'TASK1_ARRIVAL_WAITING_WAYPOINT_STATE', 'failure':'FAIL'})

        smach.StateMachine.add('TASK1_ARRIVAL_WAITING_WAYPOINT_STATE', PrepareEstimation(), transitions={'success':'TASK1_WAITING_STATE', 'fail':'FAIL'})
        smach.StateMachine.add('TASK1_WAITING_STATE', smach_ros.MonitorState("~task1_track_flag", Empty, track_flag_cb), transitions={'invalid':'TASK1_TRACKING_STATE', 'valid':'TASK1_WAITING_STATE', 'preempted':'TASK1_WAITING_STATE'})
        smach.StateMachine.add('TASK1_TRACKING_STATE', smach_ros.MonitorState("~task1_return_initial_flag", Empty, return_initial_flag_cb), transitions={'invalid':'TASK1_ENTER_STATE', 'valid':'TASK1_TRACKING_STATE', 'preempted':'TASK1_TRACKING_STATE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()

