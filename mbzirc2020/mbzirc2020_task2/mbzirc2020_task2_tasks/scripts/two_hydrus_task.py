#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from std_srvs.srv import Empty as EmptyService

from mbzirc2020_task2_common.hydrus_cooperation_interface import HydrusInterface

if __name__ == '__main__':
    rospy.init_node('two_hydrus_task')

    hydrus1 = HydrusInterface(robot_ns='hydrus1')
    hydrus2 = HydrusInterface(robot_ns='hydrus2')
    #sleep to wait for ros-clients' ready
    rospy.sleep(1)

    #start
    hydrus1.start()
    hydrus2.start()
    rospy.sleep(1)

    #set position control mode
    hydrus1.set_position_ctrl_mode()
    hydrus2.set_position_ctrl_mode()
    rospy.sleep(1)

    #takeoff
    hydrus1.takeoff()
    hydrus2.takeoff()
    rospy.sleep(1)

    #wait for hovering
    hydrus1.wait_for_hovering()
    hydrus2.wait_for_hovering()

    #weaken joint force by changing angle
    #"""
    hydrus1.set_joint_angle(0.935)
    hydrus2.set_joint_angle(0.935)
    rospy.sleep(1)
    #"""

    hydrus2.set_acceleration_ctrl_mode()

    #sensor calibration
    rospy.wait_for_service('/cfs/sensor_calib/left')
    rospy.wait_for_service('/cfs/sensor_calib/right')
    try:
        calib_service_left = rospy.ServiceProxy('/cfs/sensor_calib/left',EmptyService)
        response = calib_service_left()
    except rospy.ServiceException, e:
        print("Serivice call failed: ",e)

    try:
        calib_service_right = rospy.ServiceProxy('/cfs/sensor_calib/right',EmptyService)
        response = calib_service_right()
    except rospy.ServiceException, e:
        print("Serivice call failed: ",e)
    rospy.sleep(1)

    """
    while True:
        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        nav_msg.target_acc_x = 
        nav_msg.target_acc_y = 
        nav_msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
        hydrus2.uav_nav_pub.publish(nav_msg)

        print(len(hydrus2.sensor_data))

        rospy.sleep(1)
    """


    rospy.spin()


    
