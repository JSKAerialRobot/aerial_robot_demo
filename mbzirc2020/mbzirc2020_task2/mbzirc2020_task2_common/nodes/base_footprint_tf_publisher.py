#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform, Quaternion
import tf2_ros
import tf.transformations as tft
import ros_numpy

class BaseFootprintTfPublisher:
    def __init__(self):
        rospy.init_node('base_footprint_tf_publisher')

        self.br = tf2_ros.TransformBroadcaster()
        self.robot_ns = rospy.get_param('~robot_ns')
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)
        self.odom_sub = rospy.Subscriber('uav/baselink/odom', Odometry, self.odomCallback)

    def odomCallback(self, msg):
        trans = TransformStamped()
        trans.header = msg.header
        trans.header.frame_id = '/world'
        trans.child_frame_id = self.robot_ns + '/base_footprint'
        trans.transform.translation.x = msg.pose.pose.position.x
        trans.transform.translation.y = msg.pose.pose.position.y
        trans.transform.translation.z = 0.0

        euler_orig = tft.euler_from_quaternion(ros_numpy.numpify(msg.pose.pose.orientation))
        euler = [0] * 3
        euler[2] = euler_orig[2] + self.yaw_offset
        trans.transform.rotation = ros_numpy.msgify(Quaternion, tft.quaternion_from_euler(*euler))
        self.br.sendTransform(trans)

if __name__ == '__main__':
    node = BaseFootprintTfPublisher()
    rospy.spin()
