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
    sm_top.userdata.target_object_pos = None
    sm_top.userdata.orig_global_trans = None
    sm_top.userdata.search_count = 0
    sm_top.userdata.search_failed = False

    robot = Task2HydrusInterface()

    object_translation = rospy.get_param('~object_translation_in_link2')
    object_yaw = rospy.get_param('~object_yaw_in_link2')
    object_mass = rospy.get_param('~object_mass')
    add_object_model_func = lambda robot: addObjectToModel(robot, 'add', object_translation, object_yaw, object_mass)
    remove_object_model_func = lambda robot: addObjectToModel(robot, 'remove', object_translation, object_yaw, object_mass)

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Pick'})

        sm_pick = smach.StateMachine(outcomes=['succeeded'],
                                     input_keys=['target_object_pos'],
                                     output_keys=['target_object_pos'])

        with sm_pick:
            smach.StateMachine.add('ApproachPickArea', ApproachPickArea(robot),
                                   transitions={'succeeded':'ScanPickArea'})

            smach.StateMachine.add('ScanPickArea', ScanPickArea(robot),
                                   transitions={'succeeded': 'PickVisualServoing'},
                                   remapping={'target_object_pos':'target_object_pos'})

            smach.StateMachine.add('PickVisualServoing', PickVisualServoing(robot),
                                   transitions={'succeeded': 'Grasp'},
                                   remapping={'target_object_pos':'target_object_pos'})

            smach.StateMachine.add('Grasp', Grasp(robot, add_object_model_func),
                                   transitions={'succeeded':'succeeded',
                                                'failed':'ApproachPickArea'},
                                   remapping={'target_object_pos':'target_object_pos'})

        smach.StateMachine.add('Pick', sm_pick,
                               transitions={'succeeded':'Place'},
                               remapping={'target_object_pos':'target_object_pos'})

        sm_place = smach.StateMachine(outcomes=['finish'],
                                      input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                                      output_keys=['orig_global_trans', 'search_count', 'search_failed'])

        with sm_place:
            smach.StateMachine.add('ApproachPlaceArea', ApproachPlaceArea(robot),
                                   transitions={'succeeded':'PlaceVisualServoing'})

            smach.StateMachine.add('PlaceVisualServoing', PlaceVisualServoing(robot),
                                   transitions={'succeeded':'Ungrasp',
                                                'failed':'SearchAdjustPlace',
                                                'adjust_again':'PlaceVisualServoing'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('SearchAdjustPlace', SearchMotion(robot, 'adjust_place'),
                                   transitions={'succeeded':'PlaceVisualServoing'},
                                   remapping={'orig_global_trans':'orig_global_trans',
                                              'search_count':'search_count',
                                              'search_failed':'search_failed'})

            smach.StateMachine.add('Ungrasp', Ungrasp(robot, remove_object_model_func),
                                   transitions={'finish':'finish'})

        smach.StateMachine.add('Place', sm_place,
                               transitions={'finish':'Finish'})

        smach.StateMachine.add('Finish', Finish(robot),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
