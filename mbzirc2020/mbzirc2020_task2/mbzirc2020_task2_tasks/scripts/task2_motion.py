#!/usr/bin/env python

import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia
import tf.transformations as tft

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

class Start(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo('Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass
        return 'succeeded'

class Finish(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', [0, 0], self.robot.getBaselinkPos()[2], 0.0)
        rospy.loginfo('Landing')
        self.robot.land()
        return 'succeeded'

class MoveToGraspPosition(smach.State):
    def __init__(self, robot, grasp_z_offset, grasping_point, grasping_yaw, global_object_pos, global_object_z, preshape_joint_angle):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_count'], output_keys=['object_count'])
        self.robot = robot
        self.grasp_z_offset = grasp_z_offset
        self.global_object_pos = global_object_pos
        self.global_object_z = global_object_z
        self.preshape_joint_angle = preshape_joint_angle

        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
        #preshape
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.preshape_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)

        #calc uav target coords
        target_object_pos = self.global_object_pos[userdata.object_count]
        object_global_coords = tft.compose_matrix(translate=[target_object_pos[0], target_object_pos[1], self.global_object_z], angles=[0, 0, target_object_pos[2]])
        uav_target_coords = tft.concatenate_matrices(object_global_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getBaselinkPos()[2], uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], uav_target_pos[2] + self.grasp_z_offset, uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], uav_target_pos[2], uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class Grasp(smach.State):
    def __init__(self, robot, grasp_joint_angle, add_object_model_func):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.grasp_joint_angle = grasp_joint_angle
        self.robot = robot
        self.add_object_model_func = add_object_model_func

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.grasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        rospy.sleep(5);
        self.add_object_model_func(self.robot)
        return 'succeeded'

class MoveToPlacePosition(smach.State):
    def __init__(self, robot, global_place_channel_pos, global_place_channel_z, place_z_margin, place_z_offset, grasping_point, grasping_yaw):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

        self.robot = robot
        self.global_place_channel_pos = global_place_channel_pos
        self.global_place_channel_z = global_place_channel_z
        self.place_z_margin = place_z_margin
        self.place_z_offset = place_z_offset
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
        #calc place coords
        target_channel_pos = self.global_place_channel_pos[0]
        object_count_in_channel = (userdata.object_count % 4) + 1
        place_pos_x = target_channel_pos[0] + (target_channel_pos[2] - target_channel_pos[0]) * object_count_in_channel / 5.0
        place_pos_y = target_channel_pos[1] + (target_channel_pos[3] - target_channel_pos[1]) * object_count_in_channel / 5.0
        place_pos_yaw = (target_channel_pos[3] - target_channel_pos[1]) / (target_channel_pos[2] - target_channel_pos[0])
        place_pos_coords = tft.compose_matrix(translate=[place_pos_x, place_pos_y, self.global_place_channel_z], angles=[0, 0, place_pos_yaw])

        uav_target_coords = tft.concatenate_matrices(place_pos_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.global_place_channel_z + self.place_z_offset, self.robot.getBaselinkRPY()[2], pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], uav_target_pos[2] + self.place_z_offset, uav_target_yaw)
        #self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], uav_target_pos[2] + self.place_z_margin, uav_target_yaw)
        userdata.object_count += 1
        return 'succeeded'

class Ungrasp(smach.State):
    def __init__(self, robot, ungrasp_joint_angle, remove_object_model_func, place_z_offset, object_num):
        smach.State.__init__(self, outcomes=['finish', 'continue'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])
        self.ungrasp_joint_angle = ungrasp_joint_angle
        self.robot = robot
        self.remove_object_model_func = remove_object_model_func
        self.place_z_offset = place_z_offset
        self.object_num = object_num

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.ungrasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        self.remove_object_model_func(self.robot)
        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.robot.getBaselinkPos()[2] + self.place_z_offset, self.robot.getBaselinkRPY()[2], pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)

        if userdata.object_count == self.object_num:
            return 'finish'
        else:
            return 'continue'

def main():
    rospy.init_node('mbzirc2020_task2_motion')
    sm_top = smach.StateMachine(outcomes=['succeeded'])
    sm_top.userdata.object_count = 0

    grasp_z_offset = rospy.get_param('~grasp_z_offset')
    grasp_joint_angle = rospy.get_param('~grasp_joint_angle')

    ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle')
    preshape_joint_angle = rospy.get_param('~preshape_joint_angle')
    object_translation = rospy.get_param('~object_translation')
    object_yaw = rospy.get_param('~object_yaw')
    object_mass = rospy.get_param('~object_mass')

    object_num = rospy.get_param('~object_num')
    global_object_pos = rospy.get_param('~global_object_pos')
    global_object_z = rospy.get_param('~global_object_z')
    grasping_point = rospy.get_param('~grasping_point')
    grasping_yaw = rospy.get_param('~grasping_yaw')

    global_place_channel_pos = rospy.get_param('~global_place_channel_pos')
    global_place_channel_z = rospy.get_param('~global_place_channel_z')
    place_z_margin = rospy.get_param('~place_z_margin')
    place_z_offset = rospy.get_param('~place_z_offset')

    robot = HydrusInterface()

    add_object_model_func = lambda robot: addObjectToModel(robot, 'add', object_translation, object_yaw, object_mass)
    remove_object_model_func = lambda robot: addObjectToModel(robot, 'remove', object_translation, object_yaw, object_mass)

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Pick'})

        sm_pick = smach.StateMachine(outcomes=['pick_succeeded'], input_keys=['object_count'], output_keys=['object_count'])

        with sm_pick:
            smach.StateMachine.add('MoveToGraspPosition', MoveToGraspPosition(robot, grasp_z_offset, grasping_point, grasping_yaw, global_object_pos, global_object_z, preshape_joint_angle),
                                   transitions={'succeeded':'Grasp'})

            smach.StateMachine.add('Grasp', Grasp(robot, grasp_joint_angle, add_object_model_func),
                                   transitions={'succeeded':'pick_succeeded'})


        smach.StateMachine.add('Pick', sm_pick,
                               transitions={'pick_succeeded':'Place'})

        sm_place = smach.StateMachine(outcomes=['finish', 'continue'], input_keys=['object_count'], output_keys=['object_count'])

        with sm_place:
            smach.StateMachine.add('MoveToPlacePosition', MoveToPlacePosition(robot, global_place_channel_pos, global_place_channel_z, place_z_margin, place_z_offset, grasping_point, grasping_yaw),
                                   transitions={'succeeded':'Ungrasp'},
                                   remapping={'object_count':'object_count'})

            smach.StateMachine.add('Ungrasp', Ungrasp(robot, ungrasp_joint_angle, remove_object_model_func, place_z_offset, object_num),
                                   transitions={'finish':'finish',
                                                'continue':'continue'},
                                   remapping={'object_count':'object_count'})

        smach.StateMachine.add('Place', sm_place,
                               transitions={'finish':'Finish',
                                            'continue':'Pick'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('Finish', Finish(robot),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
