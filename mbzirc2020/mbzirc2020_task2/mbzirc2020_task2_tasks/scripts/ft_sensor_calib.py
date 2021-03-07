#!/usr/bin/env python

import rospy

from geometry_msgs.msg import WrenchStamped

from std_srvs.srv import Empty,EmptyResponse

class FT_SENSOR_CALIB:
    def __init__(self,robot_ns):
        self.ft_sensor_left_sub = rospy.Subscriber(robot_ns+'/ft_sensor_topic_left',WrenchStamped,self.SensorCallbackLeft)
        self.ft_sensor_right_sub = rospy.Subscriber(robot_ns+'/ft_sensor_topic_right',WrenchStamped,self.SensorCallbackRight)

        #use same topic name of real_machine
        self.cfs_sensor_left_pub = rospy.Publisher('/cfs/data/left',WrenchStamped,queue_size=10)
        self.cfs_sensor_right_pub = rospy.Publisher('/cfs/data/right',WrenchStamped,queue_size=10)

        self.left_sensor_calib_srv = rospy.Service('/cfs/sensor_calib/left',Empty,self.CalibrationLeft)
        self.right_sensor_calib_srv = rospy.Service('/cfs/sensor_calib/right',Empty,self.CalibrationRight)

        # last data
        self.last_force_x_left = []
        self.last_force_y_left = []
        self.last_force_z_left = []
        self.last_torque_x_left = []
        self.last_torque_y_left = []
        self.last_torque_z_left = []

        self.last_force_x_right = []
        self.last_force_y_right = []
        self.last_force_z_right = []
        self.last_torque_x_right = []
        self.last_torque_y_right = []
        self.last_torque_z_right = []

        # offset
        self.offset_force_x_left = 0
        self.offset_force_y_left = 0
        self.offset_force_z_left = 0
        self.offset_torque_x_left = 0
        self.offset_torque_y_left = 0
        self.offset_torque_z_left = 0

        self.offset_force_x_right = 0
        self.offset_force_y_right = 0
        self.offset_force_z_right = 0
        self.offset_torque_x_right = 0
        self.offset_torque_y_right = 0
        self.offset_torque_z_right = 0

    def SensorCallbackLeft(self,msg):

        self.last_force_x_left.append(msg.wrench.force.x)
        self.last_force_y_left.append(msg.wrench.force.y)
        self.last_force_z_left.append(msg.wrench.force.z)
        self.last_torque_x_left.append(msg.wrench.torque.x)
        self.last_torque_y_left.append(msg.wrench.torque.y)
        self.last_torque_z_left.append(msg.wrench.torque.z)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp.secs = msg.header.stamp.secs
        wrench_msg.header.stamp.nsecs = msg.header.stamp.nsecs
        wrench_msg.header.frame_id = msg.header.frame_id
        wrench_msg.wrench.force.x = msg.wrench.force.x - self.offset_force_x_left
        wrench_msg.wrench.force.y = msg.wrench.force.y - self.offset_force_y_left
        wrench_msg.wrench.force.z = msg.wrench.force.z - self.offset_force_z_left
        wrench_msg.wrench.torque.x = msg.wrench.torque.x - self.offset_torque_x_left
        wrench_msg.wrench.torque.y = msg.wrench.torque.y - self.offset_torque_y_left
        wrench_msg.wrench.torque.z = msg.wrench.torque.z - self.offset_torque_z_left

        self.cfs_sensor_left_pub.publish(wrench_msg)

        self.last_force_x_left = self.last_force_x_left[-100:]
        self.last_force_y_left = self.last_force_y_left[-100:]
        self.last_force_z_left = self.last_force_z_left[-100:]
        self.last_torque_x_left = self.last_torque_x_left[-100:]
        self.last_torque_y_left = self.last_torque_y_left[-100:]
        self.last_torque_z_left = self.last_torque_z_left[-100:]

    def SensorCallbackRight(self,msg):

        self.last_force_x_right.append(msg.wrench.force.x)
        self.last_force_y_right.append(msg.wrench.force.y)
        self.last_force_z_right.append(msg.wrench.force.z)
        self.last_torque_x_right.append(msg.wrench.torque.x)
        self.last_torque_y_right.append(msg.wrench.torque.y)
        self.last_torque_z_right.append(msg.wrench.torque.z)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp.secs = msg.header.stamp.secs
        wrench_msg.header.stamp.nsecs = msg.header.stamp.nsecs
        wrench_msg.header.frame_id = msg.header.frame_id
        wrench_msg.wrench.force.x = msg.wrench.force.x - self.offset_force_x_right
        wrench_msg.wrench.force.y = msg.wrench.force.y - self.offset_force_y_right
        wrench_msg.wrench.force.z = msg.wrench.force.z - self.offset_force_z_right
        wrench_msg.wrench.torque.x = msg.wrench.torque.x - self.offset_torque_x_right
        wrench_msg.wrench.torque.y = msg.wrench.torque.y - self.offset_torque_y_right
        wrench_msg.wrench.torque.z = msg.wrench.torque.z - self.offset_torque_z_right

        self.cfs_sensor_right_pub.publish(wrench_msg)

        self.last_force_x_right = self.last_force_x_right[-100:]
        self.last_force_y_right = self.last_force_y_right[-100:]
        self.last_force_z_right = self.last_force_z_right[-100:]
        self.last_torque_x_right = self.last_torque_x_right[-100:]
        self.last_torque_y_right = self.last_torque_y_right[-100:]
        self.last_torque_z_right = self.last_torque_z_right[-100:]
        

    def CalibrationLeft(self,req):

        try:
            self.offset_force_x_left = sum(self.last_force_x_left)/len(self.last_force_x_left)
            self.offset_force_y_left = sum(self.last_force_y_left)/len(self.last_force_y_left)
            self.offset_force_z_left = sum(self.last_force_z_left)/len(self.last_force_z_left) 
            self.offset_torque_x_left = sum(self.last_torque_x_left)/len(self.last_torque_x_left)
            self.offset_torque_y_left = sum(self.last_torque_y_left)/len(self.last_torque_y_left)
            self.offset_torque_z_left = sum(self.last_torque_z_left)/len(self.last_torque_z_left)
            rospy.loginfo('left sensor is calibrated')
        except:
            rospy.logwarn('no sensor data')

        return EmptyResponse()

    def CalibrationRight(self,req):

        try:
            self.offset_force_x_right = sum(self.last_force_x_right)/len(self.last_force_x_right)
            self.offset_force_y_right = sum(self.last_force_y_right)/len(self.last_force_y_right)
            self.offset_force_z_right = sum(self.last_force_z_right)/len(self.last_force_z_right) 
            self.offset_torque_x_right = sum(self.last_torque_x_right)/len(self.last_torque_x_right)
            self.offset_torque_y_right = sum(self.last_torque_y_right)/len(self.last_torque_y_right)
            self.offset_torque_z_right = sum(self.last_torque_z_right)/len(self.last_torque_z_right)
            rospy.loginfo('right sensor is calibrated')
        except:
            rospy.logwarn('no sensor data')

        return EmptyResponse()

if __name__ == '__main__':

    rospy.init_node('ft_sensor_calib')

    robot_ns = rospy.get_param('/ft_sensor_calib/robot_ns','hydrus')

    hydrus_ft_calib = FT_SENSOR_CALIB(robot_ns=robot_ns)

    rospy.spin()
