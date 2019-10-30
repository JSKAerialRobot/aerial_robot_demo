#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
import sys
from std_msgs.msg import Empty

# propeller
from spinal.msg import FourAxisCommand, ServoTorqueCmd

# joint & leg
from sensor_msgs.msg import JointState

# joint & set_attitude
from std_srvs.srv import SetBool

# joy pad
from sensor_msgs.msg import Joy

# uav odom
from nav_msgs.msg import Odometry
import tf

# rviz marker display
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# pid controller
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import *

#################
class Propeller():
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name,anonymous=True)
        # publisher
        self.uav_start_pub = rospy.Publisher("/teleop_command/start", Empty, queue_size = 1)
        self.uav_halt_pub = rospy.Publisher("/teleop_command/halt", Empty, queue_size = 1)

        self.ctrl_four_axis_pub = rospy.Publisher('/aerial_robot_control_four_axis', FourAxisCommand, queue_size = 10)

    def start(self):
        self.uav_start_pub.publish()

    def halt(self):
        self.uav_halt_pub.publish()

    def set(self,thrust):
        spinal = FourAxisCommand()
        spinal.angles = [0, 0, 0]
        spinal.base_throttle = thrust
        self.ctrl_four_axis_pub.publish(spinal)

#############
class Joint():
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name,anonymous=True)

        # publisher
        self.joint_control_pub = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size = 10)

        # subscriber
        self.joint_state_sub = rospy.Subscriber("/hydrusx/joint_states", JointState, self.jointStatusCallback)

        # service
        self.torque_enable = rospy.ServiceProxy("/hydrusx/joints/torque_enable", SetBool)

        # joint servo sensor data dictionary
        self.data = {
            "angle" : {},
            "torque": {},
        }

    def set(self, joint_value, joint_name = ["joint1", "joint3"]):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time(0)
        joint_state.name = joint_name
        joint_state.position = joint_value
        self.joint_control_pub.publish(joint_state)

    def is_convergent(self, joint_value):
        angle_diff = [self.data["angle"]["joint1"] - joint_value[0], self.data["angle"]["joint3"] - joint_value[1]]
        for i in range(len(angle_diff)):
            if abs(angle_diff[i]) > 0.05:
                self.set(joint_value)
                return False
        return True

    def jointStatusCallback(self, msg):
        self.data["angle"] = {k:v for k,v in zip(msg.name, msg.position)}
        self.data["torque"] = {k:v for k,v in zip(msg.name, msg.effort)}

###########
class Leg():
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name,anonymous=True)

        # publisher
        self.leg_ctrl_pub = rospy.Publisher('/hydrusx/legs_ctrl', JointState, queue_size = 10)

        self.servo_torque_enable_pub = rospy.Publisher('/extra_servo_torque_enable', ServoTorqueCmd, queue_size = 2)

        self.leg_id = [rospy.get_param("/hydrusx/servo_controller/legs/controller1/id"), rospy.get_param("/hydrusx/servo_controller/legs/controller2/id")]

        self.leg_command = {
            'stretch' : [0.092, 0.092],
            'shrink' : [0,0],
        }

    def set(self,command):
        if command == "free":
            spinal = ServoTorqueCmd()
            spinal.index = self.leg_id
            spinal.torque_enable = [0] * len(self.leg_id)
            self.servo_torque_enable_pub.publish(spinal)
        else:
            leg_state = JointState()
            leg_state.header.stamp = rospy.Time(0)
            leg_state.name = []
            leg_state.position = self.leg_command[command]
            self.leg_ctrl_pub.publish(leg_state)

#################
class SetAttitude:
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name, anonymous=True)

    def set(self, command):
        rospy.wait_for_service('set_attitude_control')
        try:
            set_attitude = rospy.ServiceProxy("/set_attitude_control", SetBool)
            if command == "on":
                set_attitude(True)
            elif command == "off":
                set_attitude(False)
            else:
                rospy.logerror("error when set attitude in lib.py")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s", e)

#############
class UavOdom:
    data = {
        "cog": {
            "x": None,
            "y": None,
            "z": None,
            "roll": None,
            "pitch": None,
            "yaw": None,
            "linear_vel": None,
        },
        "baselink": {
            "x": None,
            "y": None,
            "z": None,
            "roll": None,
            "pitch": None,
            "yaw": None,
            "linear_vel": None,
        },
    }
    def __init__(self, name=""):
        # init node
        self.test_flag = False
        if name != "":
            rospy.init_node(name, anonymous=True)
            time.sleep(0.5)
            self.test_flag = True

        self.uav_odom_baselink_sub = rospy.Subscriber("/uav/baselink/odom", Odometry, self.uavBaselinkOdomCallback)
        self.uav_odom_cog_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.uavCogOdomCallback)

    def uavBaselinkOdomCallback(self, msg):
        self.data["baselink"]["x"] = msg.pose.pose.position.x
        self.data["baselink"]["y"] = msg.pose.pose.position.y
        self.data["baselink"]["z"] = msg.pose.pose.position.z
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.data["baselink"]["roll"] = rpy[0]
        self.data["baselink"]["pitch"] = rpy[1]
        self.data["baselink"]["yaw"] = rpy[2]
        if self.test_flag:
            rospy.loginfo("roll:%f, pitch:%f, yaw:%f", uavOdom.data["baselink"]["roll"], uavOdom.data["baselink"]["pitch"], uavOdom.data["baselink"]["yaw"])

        self.data["baselink"]["linear_vel"] = np.array([msg.twist.twist.linear.x,
                                            msg.twist.twist.linear.y,
                                            msg.twist.twist.linear.z])

    def uavCogOdomCallback(self, msg):
        self.data["cog"]["x"] = msg.pose.pose.position.x
        self.data["cog"]["y"] = msg.pose.pose.position.y
        self.data["cog"]["z"] = msg.pose.pose.position.z
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.data["cog"]["roll"] = rpy[0]
        self.data["cog"]["pitch"] = rpy[1]
        self.data["cog"]["yaw"] = rpy[2]
        if self.test_flag:
            rospy.loginfo("roll:%f, pitch:%f, yaw:%f", uavOdom.data["cog"]["roll"], uavOdom.data["cog"]["pitch"], uavOdom.data["cog"]["yaw"])

        self.data["cog"]["linear_vel"] = np.array([msg.twist.twist.linear.x,
                                            msg.twist.twist.linear.y,
                                            msg.twist.twist.linear.z])

############
class JoyPad:
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name, anonymous=True)

        # subscriber
        self.joy_stick_sub = rospy.Subscriber('joy', Joy, self.joyCallback)

        # button data
        self.push_button_name = ["square", "cross", "circle", "triangle", "L1", "R1", "L2", "R2", "SHARE", "OPTIONS", "l_stick", "r_stick", "ps", "center"]
        self.stick_name = ["x_l", "y_l", "x_r", "L2", "R2", "y_r"]

        self.button_name = self.push_button_name + self.stick_name

        self.data = {k:v for k,v in zip(self.button_name, [float(0)]*len(self.button_name))}

    def joyCallback(self, msg):
        # get joy pad data
        ## get stick data
        for i in range(len(self.stick_name)):
            self.data[self.stick_name[i]] = msg.axes[i]

        ## get push button data
        for i in range(len(self.push_button_name)):
            self.data[self.push_button_name[i]] = msg.buttons[i]
        #pass

##################
class RvizMarker():
    id = 0
    config = {
        "red_object" : {"color":{"r":1.0,"g":0.0,"b":0.0,"a":1},"scale":{"x":0.22,"y":0.3,"z":0.3},"type":1},
        "point"     : {"color":{"r":0.0,"g":1.0,"b":0.0, "a":1},"scale":{"x":0.1,"y":0.1,"z":0.01},"type":3},
        "vector"    : {"color":{"r":0.0,"g":0.0,"b":1.0, "a":1},"scale":{"x":0.15,"y":0.5,"z":0},"type":0},
        "text" : {"color":{"r":1.0,"g":0.0,"b":0.0,"a":1},"scale":{"x":0.22,"y":0.3,"z":0.3},"type":9},
    }

    def __init__(self, name, item):
        self.pub = rospy.Publisher(name, Marker, queue_size = 10)
        self.marker_data = Marker()
        self.marker_data.header.frame_id = "world"
        self.marker_data.header.stamp = rospy.Time.now()
        self.marker_data.color.r = RvizMarker.config[item]["color"]["r"]
        self.marker_data.color.g = RvizMarker.config[item]["color"]["g"]
        self.marker_data.color.b = RvizMarker.config[item]["color"]["b"]
        self.marker_data.color.a = RvizMarker.config[item]["color"]["a"]
        self.marker_data.scale.x = RvizMarker.config[item]["scale"]["x"]
        self.marker_data.scale.y = RvizMarker.config[item]["scale"]["y"]
        self.marker_data.scale.z = RvizMarker.config[item]["scale"]["z"]
        self.marker_data.ns = "basic_shapes"
        self.marker_data.id = RvizMarker.id
        RvizMarker.id += 1
        self.marker_data.type = RvizMarker.config[item]["type"]

    def display(self, position, rpy, text = None, lifetime = None):
        self.marker_data.id = RvizMarker.id
        self.marker_data.action = Marker.ADD

        if self.marker_data.type == 0: # if arrow
            start, end = Point(), Point()
            start.x = position[0]
            start.y = position[1]
            start.z = 0
            end.x = rpy[0]
            end.y = rpy[1]
            end.z = 0
            self.marker_data.points = []
            self.marker_data.points.append(start)
            self.marker_data.points.append(end)
        elif self.marker_data.type == 9: # if text
            self.marker_data.text = text
        else:
            self.marker_data.pose.position.x = position[0]
            self.marker_data.pose.position.y = position[1]
            self.marker_data.pose.position.z = position[2]

            orientation = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            self.marker_data.pose.orientation.x = orientation[0]
            self.marker_data.pose.orientation.y = orientation[1]
            self.marker_data.pose.orientation.z = orientation[2]
            self.marker_data.pose.orientation.w = orientation[3]

        self.marker_data.lifetime = rospy.Duration() if lifetime == None else lifetime

        self.pub.publish(self.marker_data)

##################
class SetPidParam():
    def __init__(self, name=""):
        # init node
        if name != "":
            rospy.init_node(name, anonymous=True)

        self.config = Config()

        # service
        self.leg1 = rospy.ServiceProxy("/hydrusx/servo_controller/legs/controller1/simulation/pid/set_parameters", Reconfigure)
        self.leg2 = rospy.ServiceProxy("/hydrusx/servo_controller/legs/controller2/simulation/pid/set_parameters", Reconfigure)

    def set(self, name="p", value=15000):
        doubleparam = DoubleParameter()
        doubleparam.name = name
        doubleparam.value = value
        self.config.doubles = [doubleparam]

        self.leg1(self.config)
        self.leg2(self.config)

#########################
if __name__ == "__main__":
    command_line_args_list = ["propeller", "uavOdom", "switch"]
    # rosrun args
    args = rospy.myargv(argv=sys.argv)
    if args[1] == command_line_args_list[0]:
        propeller = Propeller("propeller_test")
        time.sleep(0.5)
        propeller.start()
        time.sleep(3)
        propeller.set([3,3,3,3])
        time.sleep(2)
        propeller.halt()
    elif args[1] == command_line_args_list[1]:
        uavOdom = UavOdom("uavOdom_test")
        rospy.spin()
    else:
        pass
