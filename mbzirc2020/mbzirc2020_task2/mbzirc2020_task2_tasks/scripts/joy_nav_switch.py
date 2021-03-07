#!/usr/bin/env python

import rospy

from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

from std_srvs.srv import Empty as EmptyService
from std_srvs.srv import SetBool, SetBoolRequest

class JoyControl:
    def __init__(self,robot_ns):

        self.ft_flag = 1
        self.grasp_flag = 1

        self.prev_joy_state = Joy()

        self.joy_sub = rospy.Subscriber(robot_ns + '/joy',Joy, self.JoyStickCallback)

        self.switch_pub = rospy.Publisher(robot_ns + '/ft_sensor_feedback_switch', Int8, queue_size=1)
        self.joint_pub = rospy.Publisher(robot_ns + '/joints_ctrl', JointState, queue_size=1)

        rospy.wait_for_service('/cfs/sensor_calib/left')
        self.calib_left = rospy.ServiceProxy('/cfs/sensor_calib/left',EmptyService)
        rospy.wait_for_service('/cfs/sensor_calib/right')
        self.calib_right = rospy.ServiceProxy('/cfs/sensor_calib/right',EmptyService)
        rospy.wait_for_service(robot_ns + '/joints/torque_enable')
        self.set_joint_torque_client = rospy.ServiceProxy(robot_ns + '/joints/torque_enable', SetBool)

    def JoyStickCallback(self,msg):
        if msg.buttons[7] == 1 and self.prev_joy_state.buttons[7] == 0:

            self.calib_left()
            self.calib_right()

            rospy.loginfo('calibrated by joy-stick')
        elif msg.buttons[4] == 1 and self.prev_joy_state.buttons[4] == 0:

            switch_msg = Int8()
            switch_msg.data = self.ft_flag
            self.switch_pub.publish(switch_msg)

            self.ft_flag = 1 - self.ft_flag

            rospy.loginfo('swithced ft navigation by joy-stick')
        elif msg.buttons[5] == 1 and self.prev_joy_state.buttons[5] == 0:
            if self.grasp_flag == 1:
                angle = 1.0
                state = True

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                rospy.sleep(0.5)

                joint_msg = JointState()
                joint_msg.name = ['joint1','joint3']
                joint_msg.position = [angle,angle]
                joint_msg.velocity = [0.0]
                joint_msg.effort = [0.0]

                for i in range(100):
                    self.joint_pub.publish(joint_msg)
                    rospy.sleep(0.01)
            else:
                angle = 0.7
                state = False

                joint_msg = JointState()
                joint_msg.name = ['joint1','joint3']
                joint_msg.position = [angle,angle]
                joint_msg.velocity = [0.0]
                joint_msg.effort = [0.0]

                for i in range(100):
                    self.joint_pub.publish(joint_msg)
                    rospy.sleep(0.01)

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                rospy.sleep(0.5)

            self.grasp_flag = 1 - self.grasp_flag

            rospy.loginfo('grasping command by joy-stick')

        self.prev_joy_state = msg

if __name__=="__main__":

    rospy.init_node('joy_nav_switch')

    robot_ns = rospy.get_param('/joy_nav_switch/robot_ns','hydrus')

    joycontrol = JoyControl(robot_ns = robot_ns)

    rospy.spin()

