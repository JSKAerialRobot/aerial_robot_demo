#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int8
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from dynamic_reconfigure.msg import Config,BoolParameter,DoubleParameter

from dynamic_reconfigure.srv import Reconfigure

class TaskClient:
    def __init__(self,robot_ns_master,robot_ns_slave,master_des,slave_des):

        self.master_des = master_des
        self.slave_des = slave_des

        self.master_uav_nav_pub = rospy.Publisher(robot_ns_master + '/uav/nav', FlightNav, queue_size=10)
        self.slave_uav_nav_pub = rospy.Publisher(robot_ns_slave + '/uav/nav', FlightNav, queue_size=10)

        self.slave_ft_sensor_feedback_switch_pub = rospy.Publisher(robot_ns_slave + '/ft_sensor_feedback_switch', Int8, queue_size=150)

        self.master_odom_sub = rospy.Subscriber(robot_ns_master +'/uav/cog/odom', Odometry, self.MasterOdometryCallback)
        self.slave_odom_sub = rospy.Subscriber(robot_ns_slave +'/uav/cog/odom', Odometry, self.SlaveOdometryCallback)

        self.master_set_yaw_free_service = rospy.ServiceProxy(robot_ns_master + '/controller/lqi/set_parameters',Reconfigure)
        self.slave_set_yaw_free_service = rospy.ServiceProxy(robot_ns_slave + '/controller/lqi/set_parameters',Reconfigure)

        self.master_position = np.array([0,0,0])
        self.slave_position = np.array([0,0,0])

    def MasterOdometryCallback(self,msg):

        self.master_position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])

    def SlaveOdometryCallback(self,msg):

        self.slave_position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])

    def master_movement(self):

        position = self.master_position

        x = np.linspace(position[0],self.master_des[0],100)
        y = np.linspace(position[1],self.master_des[1],100)

        for i in range(100):
            nav_msg = FlightNav()
            nav_msg.header.frame_id='global'
            # LOCAL
            nav_msg.control_frame=0
            # COG
            nav_msg.target=1
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = x[i]
            nav_msg.target_pos_y = y[i]
            self.master_uav_nav_pub.publish(nav_msg)

            rospy.sleep(0.1)

        print('master nav')

    def slave_movement(self):

        position = self.slave_position

        x = np.linspace(position[0],self.slave_des[0],30)
        y = np.linspace(position[1],self.slave_des[1],30)

        for i in range(30):
            nav_msg = FlightNav()
            nav_msg.header.frame_id='global'
            # LOCAL
            nav_msg.control_frame=0
            # COG
            nav_msg.target=1
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = x[i]
            nav_msg.target_pos_y = y[i]
            self.slave_uav_nav_pub.publish(nav_msg)

            rospy.sleep(0.1)

        print('slave nav')


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

    rospy.init_node('short_demo')

    robot_ns_master = 'hydrus1'
    robot_ns_slave = 'hydrus2'

    task_client = TaskClient(robot_ns_master=robot_ns_master,
                        robot_ns_slave=robot_ns_slave,
                        master_des=[1.73,0.54],
                        slave_des=[-0.38,0.62])

    rospy.sleep(1)

    rospy.loginfo('master yaw free')
    task_client.master_set_yaw_free(flag = True)

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 1
    rospy.loginfo('slave feedback switch on')
    task_client.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)

    rospy.sleep(1)

    task_client.master_movement()

    rospy.sleep(1)

    switch_msg = Int8()
    switch_msg.data = 0
    rospy.loginfo('slave feedback switch off')
    task_client.slave_ft_sensor_feedback_switch_pub.publish(switch_msg)

    rospy.sleep(0.5)

    rospy.loginfo('slave yaw free')
    task_client.slave_set_yaw_free(flag = True)

    rospy.sleep(1)

    task_client.slave_movement()
