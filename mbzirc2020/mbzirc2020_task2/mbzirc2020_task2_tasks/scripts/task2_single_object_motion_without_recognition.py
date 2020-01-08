#!/usr/bin/env python

import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia

def addObjectToModel(robot, action):
    tf = Transform()
    tf.translation.x = 0.1
    tf.translation.y = 0.5
    tf.translation.z = 0.0
    tf.rotation.x = 0.0
    tf.rotation.y = 0.0
    tf.rotation.z = -0.3824995
    tf.rotation.w = 0.9239557
    inertia = Inertia()
    inertia.m = 1.0
    inertia.com.x = 0.0
    inertia.com.y = 0.0
    inertia.com.z = 0.0
    inertia.ixx = 0.00664
    inertia.iyy = 0.01079
    inertia.izz = 0.01079

    robot.addExtraModule(action, 'object', 'link2', tf, inertia)

class Start(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo('Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        rospy.sleep(20)
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
    def __init__(self, robot, grasp_pos, grasp_z_offset):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.grasp_xy = [grasp_pos[0], grasp_pos[1]]
        self.grasp_z = grasp_pos[2]
        self.grasp_yaw = grasp_pos[3]
        self.grasp_z_offset = grasp_z_offset
        self.robot = robot

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', self.grasp_xy, self.grasp_z + self.grasp_z_offset, self.grasp_yaw)
        self.robot.goPosWaitConvergence('global', self.grasp_xy, self.grasp_z, self.grasp_yaw)

        return 'succeeded'

class Grasp(smach.State):
    def __init__(self, robot, grasp_joint_angle):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.grasp_joint_angle = grasp_joint_angle
        self.robot = robot

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.grasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        addObjectToModel(self.robot, 'add')
        rospy.sleep(10);
        return 'succeeded'

class MoveToPlacePosition(smach.State):
    def __init__(self, robot, place_pos, place_z_offset):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

        self.place_xy = [place_pos[0], place_pos[1]]
        self.place_z = place_pos[2]
        self.place_yaw = place_pos[3]
        self.place_z_offset = place_z_offset
        self.robot = robot

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', self.place_xy, self.place_z + self.place_z_offset, self.place_yaw)
        self.robot.goPosWaitConvergence('global', self.place_xy, self.place_z + self.place_z_offset, self.place_yaw)
        userdata.object_count += 1
        return 'succeeded'

class Ungrasp(smach.State):
    def __init__(self, robot, ungrasp_joint_angle):
        smach.State.__init__(self, outcomes=['succeeded', 'finish'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])
        self.ungrasp_joint_angle = ungrasp_joint_angle
        self.robot = robot

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.ungrasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        if userdata.object_count == 1:
            addObjectToModel(self.robot, 'remove')

        if userdata.object_count == 0:
            return 'succeeded'
        else:
            return 'finish'

def main():
    rospy.init_node('mbzirc2020_task2_motion')
    sm = smach.StateMachine(outcomes=['succeeded'])
    sm.userdata.object_count = 0

    grasp_pos = rospy.get_param('~grasp_pos')
    grasp_z_offset = rospy.get_param('~grasp_z_offset')
    grasp_joint_angle = rospy.get_param('~grasp_joint_angle')
    place_pos = rospy.get_param('~place_pos')
    place_z_offset = rospy.get_param('~place_z_offset')
    ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle')

    robot = HydrusInterface()

    with sm:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Ungrasp'})

        smach.StateMachine.add('Ungrasp', Ungrasp(robot, ungrasp_joint_angle),
                               transitions={'succeeded':'MoveToGraspPosition',
                                            'finish':'Finish'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('MoveToGraspPosition', MoveToGraspPosition(robot, grasp_pos, grasp_z_offset),
                               transitions={'succeeded':'Grasp'})

        smach.StateMachine.add('Grasp', Grasp(robot, grasp_joint_angle),
                               transitions={'succeeded':'MoveToPlacePosition'})

        smach.StateMachine.add('MoveToPlacePosition', MoveToPlacePosition(robot, place_pos, place_z_offset),
                               transitions={'succeeded':'Ungrasp'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('Finish', Finish(robot),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm, '/SM_ROOT')
        sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
