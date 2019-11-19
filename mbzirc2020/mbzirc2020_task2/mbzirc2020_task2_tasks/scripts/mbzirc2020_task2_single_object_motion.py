#!/usr/bin/env python

import rospy
import smach
import smach_ros
from hydrus import hydrus_interface
from sensor_msgs.msg import JointState
import numpy as np

robot = hydrus_interface.HydrusInterface()

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Takeoff')
        rospy.sleep(0.5)
        robot.startAndTakeoff()
        rospy.sleep(30)
        return 'succeeded'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Landing')
        robot.land()
        return 'succeeded'

class MoveToGraspPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        robot.goPosWaitConvergence('global', [0.8, 1], 1, np.pi/4)
        robot.goPosWaitConvergence('global', [0.8, 1], 0.17, np.pi/4)

        return 'succeeded'

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = [1.3, 1.3]
        robot.setJointAngle(joint_state, time = 3000)
        return 'succeeded'

class MoveToPlacePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

    def execute(self, userdata):
        robot.goPosWaitConvergence('global', [0.8, 0], 1, np.pi/4)
        robot.goPosWaitConvergence('global', [0.8, 0], 0.17, np.pi/4)
        userdata.object_count += 1
        return 'succeeded'

class Ungrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'finish'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = [0.8, 0.8]
        robot.setJointAngle(joint_state, time = 3000)

        if userdata.object_count == 0:
            return 'succeeded'
        else:
            return 'finish'

def main():
    rospy.init_node('mbzirc2020_task2_motion')
    sm = smach.StateMachine(outcomes=['succeeded'])
    sm.userdata.object_count = 0

    with sm:
        smach.StateMachine.add('Start', Start(),
                               transitions={'succeeded':'Ungrasp'})

        smach.StateMachine.add('Ungrasp', Ungrasp(),
                               transitions={'succeeded':'MoveToGraspPosition',
                                            'finish':'Finish'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('MoveToGraspPosition', MoveToGraspPosition(),
                               transitions={'succeeded':'Grasp'})

        smach.StateMachine.add('Grasp', Grasp(),
                               transitions={'succeeded':'MoveToPlacePosition'})

        smach.StateMachine.add('MoveToPlacePosition', MoveToPlacePosition(),
                               transitions={'succeeded':'Ungrasp'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('Finish', Finish(),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm, '/SM_ROOT')
        sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
