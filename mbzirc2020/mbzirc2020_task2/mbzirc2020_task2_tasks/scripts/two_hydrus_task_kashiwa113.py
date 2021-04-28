#!/usr/bin/env python

import rospy

import tf2_ros
import tf.transformations as tft
import ros_numpy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from std_srvs.srv import Empty as EmptyService

from task2_hydrus_interface import Task2HydrusInterface

class TaskStart:

    def __init__(self):
        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.TaskStartCallback)

        self.task_start = False

    def TaskStartCallback(self,msg):
        self.task_start = True

if __name__ == '__main__':

    rospy.init_node('two_hydrus_task')

    task_start = TaskStart()

    hydrus1 = Task2HydrusInterface(robot_ns='hydrus1')
    hydrus2 = Task2HydrusInterface(robot_ns='hydrus2')
    #sleep to wait for ros-clients' ready
    rospy.sleep(1)

    while not task_start.task_start:
        rospy.sleep(1.0)

    #start
    hydrus1.start()
    hydrus2.start()
    rospy.sleep(1)

    #takeoff
    hydrus1.takeoff()
    hydrus2.takeoff()
    rospy.sleep(1)

    #wait for hovering
    rospy.loginfo("wait for hovering")
    hydrus1.wait_for_hovering()
    hydrus2.wait_for_hovering()
    rospy.loginfo("complete: wait for hovering")
    rospy.sleep(1)

    #change the follower machine to vel-nav mode
    rospy.loginfo("change the follower machine to vel-nav mode")
    hydrus2.change_ctrl_mode(mode='vel')
    rospy.loginfo("complete: change the follower machine to vel-nav mode")
    rospy.sleep(1)

    #set robots to yaw_free_ctrl
    rospy.loginfo("set robots to yaw_free_ctrl")
    hydrus1.set_yaw_free(flag=True)
    hydrus2.set_yaw_free(flag=True)
    rospy.loginfo("complete: set robots to yaw_free_ctrl")
    rospy.sleep(1)

    #the leader machine approaches the place position
    hydrus1.goPosWaitConvergence('global', [-1.47,-1.57], None, None, timeout=10, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

    #change the leader machine to pos-nav mode
    rospy.loginfo("change the leader machine to pos-nav mode")
    hydrus1.change_ctrl_mode(mode='pos')
    rospy.loginfo("complete: change the leader machine to pos-nav mode")
    rospy.sleep(1)

    #the follower machine approaches the place position
    rospy.loginfo("the follower machine approaches the place position")
    hydrus2.goPosWaitConvergence('global', [-1.47,0.43], None, None, timeout=10, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
    rospy.loginfo("complete: the follower machine approaches the place position")
    rospy.sleep(1)

    #ungrasp
    rospy.loginfo("ungrasp")
    hydrus1.ungrasp(time=500)
    hydrus2.ungrasp(time=500)
    rospy.loginfo("complete: ungrasp")
    rospy.sleep(1)

    #set robots  yaw_ctrl
    rospy.loginfo("set robots yaw_ctrl")
    hydrus1.set_yaw_free(flag=False)
    hydrus2.set_yaw_free(flag=False)
    rospy.loginfo("complete set robots yaw_ctrl")
    rospy.sleep(5)

    # back to start position
    rospy.loginfo("back to start position")
    hydrus2.goPosWaitConvergence('global', [0.74,0.16], None, None, timeout=10, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
    rospy.sleep(10)
    hydrus1.goPosWaitConvergence('global', [-0.6,-1.4], None, None, timeout=10, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
    rospy.loginfo("complete: back to start position")

    #land
    rospy.loginfo("land")
    hydrus1.land()
    hydrus2.land()
    rospy.loginfo("complete: land")
    rospy.sleep(1)

    rospy.loginfo("complete: all_tasks")
