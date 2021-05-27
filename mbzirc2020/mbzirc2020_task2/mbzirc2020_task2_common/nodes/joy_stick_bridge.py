#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

from std_srvs.srv import SetBool, SetBoolRequest
from dynamic_reconfigure.msg import Config,BoolParameter,DoubleParameter

from dynamic_reconfigure.srv import Reconfigure

class JoyStickBridge:
    def __init__(self,robot_ns_master,robot_ns_slave):
        self.grasp_flag = 1

        self.prev_joy_state = Joy()

        self.start_pub = rospy.Publisher(robot_ns_slave + '/teleop_command/start', Empty, queue_size=10)
        self.takeoff_pub = rospy.Publisher(robot_ns_slave + '/teleop_command/takeoff', Empty, queue_size=10)

        self.halt_pub = rospy.Publisher(robot_ns_slave + '/teleop_command/halt', Empty, queue_size=10)
        self.landing_pub = rospy.Publisher(robot_ns_slave + '/teleop_command/land', Empty, queue_size=10)
        self.joint_slave_pub = rospy.Publisher(robot_ns_slave + '/joints_ctrl', JointState, queue_size=1)

        self.joint_master_pub = rospy.Publisher(robot_ns_master + '/joints_ctrl', JointState, queue_size=1)
        self.leader_ctrl_mode_pub = rospy.Publisher(robot_ns_master+ '/teleop_command/ctrl_mode', Int8, queue_size=10)
        self.follower_ctrl_mode_pub = rospy.Publisher(robot_ns_slave+ '/teleop_command/ctrl_mode', Int8, queue_size=10)



        self.joy_from_master_sub = rospy.Subscriber(robot_ns_master + '/joy',Joy, self.JoyStickBridgeCallback)


        #rospy.wait_for_service(robot_ns_slave + '/joints/torque_enable')
        self.set_joint_torque_slave_client = rospy.ServiceProxy(robot_ns_slave + '/joints/torque_enable', SetBool)
        #rospy.wait_for_service(robot_ns_master + '/joints/torque_enable')
        self.set_joint_torque_master_client = rospy.ServiceProxy(robot_ns_master + '/joints/torque_enable', SetBool)
        self.master_set_yaw_free_service = rospy.ServiceProxy(robot_ns_master + '/controller/lqi/set_parameters',Reconfigure)
        self.slave_set_yaw_free_service = rospy.ServiceProxy(robot_ns_slave + '/controller/lqi/set_parameters',Reconfigure)

    def JoyStickBridgeCallback(self,msg):

        #rotor arming
        if msg.buttons[9] == 1 and self.prev_joy_state.buttons[9] == 0:
            self.start_pub.publish(Empty())
            pass

        #takeoff
        elif (msg.buttons[2] == 1 and msg.axes[9] == 1.0) and not (self.prev_joy_state.buttons[2] == 1 and self.prev_joy_state.axes[9] == 1.0):
            self.takeoff_pub.publish(Empty())
            pass

        #halt
        elif (msg.buttons[8] == 1 and self.prev_joy_state.buttons[8] == 0) or (msg.buttons[8] == 1 and self.prev_joy_state.buttons[8] == 1):
            self.halt_pub.publish(Empty())

        #landing
        elif (msg.buttons[0] == 1 and msg.axes[9] == -1.0) and not (self.prev_joy_state.buttons[0] == 1 and self.prev_joy_state.axes[9] == -1.0):
            self.landing_pub.publish(Empty())


        #in emergency change to pos-mode & ungrasp 
        elif msg.buttons[4] == 1 and self.prev_joy_state.buttons[4] == 0:

            angle = 0.7
            joint_msg = JointState()
            joint_msg.name = ['joint1','joint3']
            joint_msg.position = [angle,angle]
            joint_msg.velocity = [0.0]
            joint_msg.effort = [0.0]

            for i in range(100):
                self.joint_master_pub.publish(joint_msg)
                self.joint_slave_pub.publish(joint_msg)
                rospy.sleep(0.01)

            msg = Int8()
            msg.data = 0
            for i in range(5):
                self.leader_ctrl_mode_pub.publish(msg)
                self.follower_ctrl_mode_pub.publish(msg)
                rospy.sleep(0.1)

        #grasping
        elif msg.buttons[5] == 1 and self.prev_joy_state.buttons[5] == 0:
            if self.grasp_flag == 1:
                state = True

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_slave_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_master_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                rospy.sleep(0.5)

                angle = 1.15
                joint_msg = JointState()
                joint_msg.name = ['joint1','joint3']
                joint_msg.position = [angle,angle]
                joint_msg.velocity = [0.0]
                joint_msg.effort = [0.0]

                for i in range(100):
                    self.joint_master_pub.publish(joint_msg)
                    self.joint_slave_pub.publish(joint_msg)
                    rospy.sleep(0.01)

            else:
                state = False

                angle = 0.7
                joint_msg = JointState()
                joint_msg.name = ['joint1','joint3']
                joint_msg.position = [angle,angle]
                joint_msg.velocity = [0.0]
                joint_msg.effort = [0.0]

                for i in range(100):
                    self.joint_master_pub.publish(joint_msg)
                    self.joint_slave_pub.publish(joint_msg)
                    rospy.sleep(0.01)

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_slave_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                req = SetBoolRequest()
                req.data = state
                try:
                    self.set_joint_torque_master_client(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                rospy.sleep(0.5)

            self.grasp_flag = 1 - self.grasp_flag

            rospy.loginfo('grasping command by joy-stick')


        self.prev_joy_state = msg

    def master_set_yaw_free(self,flag=True):
        set_yaw_free_srv = Config()
        bool_param = BoolParameter()
        bool_param.name = 'lqi_flag'
        bool_param.value = True
        set_yaw_free_srv.bools = [bool_param]
        if flag == True:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 0.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.0
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 0.0
            doubles = [doubles1,doubles2,doubles3]
        elif flag == False:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 100.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.05
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 50.0
            doubles = [doubles1,doubles2,doubles3]

        for d in doubles:
            set_yaw_free_srv.doubles = [d]
            self.master_set_yaw_free_service(set_yaw_free_srv)

    def slave_set_yaw_free(self,flag=True):
        set_yaw_free_srv = Config()
        bool_param = BoolParameter()
        bool_param.name = 'lqi_flag'
        bool_param.value = True
        set_yaw_free_srv.bools = [bool_param]
        if flag == True:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 0.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.0
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 0.0
            doubles = [doubles1,doubles2,doubles3]
        elif flag == False:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 100.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.05
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 50.0
            doubles = [doubles1,doubles2,doubles3]

        for d in doubles:
            set_yaw_free_srv.doubles = [d]
            self.slave_set_yaw_free_service(set_yaw_free_srv)



if __name__=="__main__":

    rospy.init_node('joy_stick_bridge')

    robot_ns_master = rospy.get_param('/joy_stick_bridge/robot_ns_master','hydrus1')
    robot_ns_slave = rospy.get_param('/joy_stick_bridge/robot_ns_slave','hydrus2')

    joy_stick_bridge = JoyStickBridge(robot_ns_master,robot_ns_slave)


    rospy.spin()
