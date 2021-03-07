#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int8
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from dynamic_reconfigure.msg import Config,BoolParameter,DoubleParameter

from dynamic_reconfigure.srv import Reconfigure

class Master:
    def __init__(self,robot_ns):

        self.uav_nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=10)

        self.slave_ft_sensor_feedback_switch_pub = rospy.Publisher('/hydrus2/ft_sensor_feedback_switch', Int8, queue_size=150)

        self.odom_sub = rospy.Subscriber(robot_ns +'/uav/cog/odom', Odometry, self.OdometryCallback)

        self.set_yaw_free_service = rospy.ServiceProxy(robot_ns + '/controller/lqi/set_parameters',Reconfigure)

        self.position = np.array([0,0,0])

    def OdometryCallback(self,msg):

        self.position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])



    def yaw_movement(self):

        position = self.position

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]-0.1
        nav_msg.target_pos_y = position[1]-0.5
        #nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        #nav_msg.target_yaw = target_yaw
        self.uav_nav_pub.publish(nav_msg)

        print('nav 1')

        rospy.sleep(3)

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]-1.0
        #nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        #nav_msg.target_yaw = target_yaw
        self.uav_nav_pub.publish(nav_msg)

        print('nav 2')

        rospy.sleep(3)

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]-0.1
        nav_msg.target_pos_y = position[1]-0.5
        #nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        #nav_msg.target_yaw = target_yaw
        self.uav_nav_pub.publish(nav_msg)

        print('nav 3')

        rospy.sleep(3)

        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]
        #nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        #nav_msg.target_yaw = target_yaw
        self.uav_nav_pub.publish(nav_msg)

        print('nav 4')

        rospy.sleep(2)

    def set_yaw_free(self,flag=True):
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
            self.set_yaw_free_service(set_yaw_free_srv)

if __name__=="__main__":

    rospy.init_node('master_X_axis_movement')

    robot_ns_master = 'hydrus1'

    master = Master(robot_ns=robot_ns_master)

    rospy.sleep(1)

    master.set_yaw_free(flag = True)

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 1
    master.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)

    rospy.sleep(1)

    master.yaw_movement()

    rospy.sleep(1)

    master.set_yaw_free(flag = False)

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 0
    master.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)
