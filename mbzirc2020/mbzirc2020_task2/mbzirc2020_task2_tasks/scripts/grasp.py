#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys, time
from mylib import Propeller, Joint
from std_msgs.msg import String


class HydrusGraspingOnGround(object):
    thrust = {
        'joint1':[0,0,0,0],
        'joint3':[0,0,0,0],
        'free':[2,2,2,2],
    }

    angle = {
        "delta" : 0.01
    }

    def __init__(self, name=""):
        # common when test this python script or not
        self.propeller = Propeller()
        self.joint = Joint()

        self.message = ""

        self.message_pub = rospy.Publisher("/message", String, queue_size=1)
        self.message_sub = rospy.Subscriber("/message", String, self.messageCallback, queue_size=1, buff_size=1)
        
        # when test this python script or not
        if name != "":
            rospy.init_node(name, anonymous=True)
            time.sleep(0.5)
            control_rate = rospy.get_param('~control_rate', 20.0)
            rospy.Timer(rospy.Duration(1.0/control_rate), self.controlCallback)
        else:
            pass

        
        rospy.Timer(rospy.Duration(0.5), self.controlCallback)

        
    def grasp(self, joint_name, torque = 0):
        if (self.joint.data["torque"][joint_name] > torque) or (self.joint.data["angle"][joint_name] >= 1.56):
            return True
        else :
            # angle -> angle + delta
            self.joint.set([self.joint.data["angle"][joint_name] + HydrusGraspingOnGround.angle["delta"]], [joint_name])
            return False

        
    def messageCallback(self, msg):
        self.message = msg.data
            
    def controlCallback(self, event):

        if self.message == "target reached":
            if not self.grasp("joint3", 3.0) and not self.grasp("joint1", 3.0):
                rospy.loginfo("joints grasping")
                return
        
            else:
                msg = String()
                msg.data = "grasped"
                self.message_pub.publish(msg)
            
                rospy.loginfo("finish grasping")
                return

class HydrusGraspingOnGroundOnSimulator(HydrusGraspingOnGround):
    pass


if __name__ == '__main__':
    try:
        # rosrun args
        args = rospy.myargv(argv=sys.argv)
        hydrus_grasping_on_ground = HydrusGraspingOnGround("hydrus_moving_on_ground_test") if args=="" else HydrusGraspingOnGroundOnSimulator("hydrus_moving_on_ground_test")
        time.sleep(0.5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
