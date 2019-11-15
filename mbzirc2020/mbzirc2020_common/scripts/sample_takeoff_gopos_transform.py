#!/usr/bin/env python

import rospy
from hydrus.hydrus_interface import *
from sensor_msgs.msg import JointState

rospy.init_node("hydrus_interface")
robot = HydrusInterface()
rospy.sleep(1)

robot.startAndTakeoff()
rospy.sleep(20)

result = robot.goPosWaitConvergence('global', [3, 3], 2, 0.2)
print(result)

rospy.sleep(2)

joint_state = JointState()
joint_state.name = ['joint1', 'joint3']
joint_state.position = [1.0, 1.0]
robot.setJointAngle(joint_state, time = 5000)
