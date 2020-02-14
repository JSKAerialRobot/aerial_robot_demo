#!/usr/bin/env python

import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from task2_hydrus_interface import Task2HydrusInterface
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray
import tf.transformations as tft
import tf2_ros
import ros_numpy
from std_msgs.msg import UInt8, Empty
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
import copy
from task2_state import *

def addObjectToModel(robot, action, translation, yaw, mass):
    transform = Transform()
    transform.translation.x = translation[0]
    transform.translation.y = translation[1]
    transform.translation.z = translation[2]
    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    inertia = Inertia()
    inertia.m = mass
    inertia.com.x = 0.0
    inertia.com.y = 0.0
    inertia.com.z = 0.0
    inertia.ixx = 1/6.0 * mass * (0.2**2 + 0.2**2)
    inertia.iyy = 1/6.0 * mass * (0.2**2 + 0.3**2)
    inertia.izz = 1/6.0 * mass * (0.2**2 + 0.3**2)

    robot.addExtraModule(action, 'object', 'link2', transform, inertia)

def main():
    rospy.init_node('mbzirc2020_task2_motion')
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    single_object_mode = rospy.get_param('~single_object_mode', True)

    if single_object_mode:
        target_object_number = rospy.get_param('~target_object_number')
        sm_top.userdata.object_count = target_object_number
        rospy.logerr('SINGLE OBJECT MODE')
        rospy.logerr('target object number: %d', target_object_number)
    else:
        sm_top.userdata.object_count = 0
        rospy.logerr('MULTI OBJECT MODE')
        object_num = rospy.get_param("~object_num")
        rospy.logerr('object num: %d', object_num)

    sm_top.userdata.orig_global_trans = None
    sm_top.userdata.search_count = 0
    sm_top.userdata.search_failed = False

    robot = Task2HydrusInterface()

    object_translation = rospy.get_param('~object_translation')
    object_yaw = rospy.get_param('~object_yaw')
    object_mass = rospy.get_param('~object_mass')
    do_object_recognition = rospy.get_param('~do_object_recognition')
    do_channel_recognition = rospy.get_param('~do_channel_recognition')

    if not do_object_recognition:
        rospy.logerr('WARNING!! NO OBJECT RECOGNITION')
    if not do_channel_recognition:
        rospy.logerr('WARNING!! NO CHANNEL RECOGNITION')

    add_object_model_func = lambda robot: addObjectToModel(robot, 'add', object_translation, object_yaw, object_mass)
    remove_object_model_func = lambda robot: addObjectToModel(robot, 'remove', object_translation, object_yaw, object_mass)

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Pick'})

        sm_pick = smach.StateMachine(outcomes=['succeeded'],
                                     input_keys=['object_count', 'orig_global_trans', 'search_count', 'search_failed'],
                                     output_keys=['object_count', 'orig_global_trans', 'search_count', 'search_failed'])

        with sm_pick:
            smach.StateMachine.add('ApproachPickPosition', ApproachPickPosition(robot),
                                   transitions={'succeeded':'LookDown'})

            smach.StateMachine.add('LookDown', LookDown(robot),
                                   transitions={'succeeded':'AdjustGraspPosition',
                                                'failed':'SearchLookDown'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('SearchLookDown', SearchMotion(robot, 'lookdown'),
                                   transitions={'succeeded':'LookDown'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('AdjustGraspPosition', AdjustGraspPosition(robot),
                                   transitions={'succeeded':'Grasp',
                                                'failed':'SearchAdjustGrasp',
                                                'adjust_again':'AdjustGraspPosition'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('SearchAdjustGrasp', SearchMotion(robot, 'adjust_grasp'),
                                   transitions={'succeeded':'AdjustGraspPosition'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('Grasp', Grasp(robot, add_object_model_func),
                                   transitions={'succeeded':'succeeded',
                                                'failed':'ApproachPickPosition'},
                                   remapping={'object_count':'object_count'})

        smach.StateMachine.add('Pick', sm_pick,
                               transitions={'succeeded':'Place'},
                               remapping={'object_count':'object_count',
                                          'orig_global_trans':'orig_global_trans',
                                          'search_count':'search_count',
                                          'search_failed':'search_failed'})

        sm_place = smach.StateMachine(outcomes=['finish', 'continue'],
                                      input_keys=['object_count', 'orig_global_trans', 'search_count', 'search_failed'],
                                      output_keys=['object_count', 'orig_global_trans', 'search_count', 'search_failed'])
        sm_place.userdata.orig_channel_xy_yaw = None

        with sm_place:
            smach.StateMachine.add('ApproachPlacePosition', ApproachPlacePosition(robot),
                                   transitions={'succeeded':'AdjustPlacePosition'},
                                   remapping={'object_count':'object_count',
                                              'orig_channel_xy_yaw':'orig_channel_xy_yaw'})

            smach.StateMachine.add('AdjustPlacePosition', AdjustPlacePosition(robot),
                                   transitions={'succeeded':'Ungrasp',
                                                'failed':'SearchAdjustPlace',
                                                'adjust_again':'AdjustPlacePosition'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('SearchAdjustPlace', SearchMotion(robot, 'adjust_place'),
                                   transitions={'succeeded':'AdjustPlacePosition'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('Ungrasp', Ungrasp(robot, remove_object_model_func),
                                   transitions={'finish':'finish',
                                                'continue':'continue'},
                                   remapping={'object_count':'object_count',
                                              'orig_channel_xy_yaw':'orig_channel_xy_yaw'})

        smach.StateMachine.add('Place', sm_place,
                               transitions={'finish':'Finish',
                                            'continue':'Pick'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('Finish', Finish(robot),
                               transitions={'succeeded':'succeeded'},
                               remapping={'object_count':'object_count',
                                          'orig_global_trans':'orig_global_trans',
                                          'search_count':'search_count',
                                          'search_failed':'search_failed'})


        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
