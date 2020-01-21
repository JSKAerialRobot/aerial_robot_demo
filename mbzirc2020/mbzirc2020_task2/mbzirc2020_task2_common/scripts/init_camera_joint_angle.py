#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math

class InitCameraJointAngle:
    def __init__(self):
        rospy.init_node('init_camera_joint_angle')

        self.init_angle = None

        while self.init_angle is None:
            try:
                self.init_angle = rospy.get_param('/hydrusx/servo_controller/extra_servos/simulation/init_value')
            except:
                pass

        self.pub = rospy.Publisher('/hydrusx/extra_servos_ctrl', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/hydrusx/joint_states', JointState, self.callback)
        self.complete = False

    def callback(self, msg):
        pub_msg = JointState()
        pub_msg.name = ["rs_d435_servo_joint"]
        pub_msg.position = [self.init_angle]
        self.complete = True
        if not rospy.is_shutdown():
            self.pub.publish(pub_msg)

if __name__=='__main__':
    node = InitCameraJointAngle()

    while not rospy.is_shutdown():
        if node.complete:
            rospy.sleep(10)
            rospy.signal_shutdown('init gimbal angles complete')
