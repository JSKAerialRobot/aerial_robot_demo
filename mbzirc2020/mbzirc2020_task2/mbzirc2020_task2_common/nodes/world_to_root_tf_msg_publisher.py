#!/usr/bin/env python

import rospy
import tf2_ros
import ros_numpy
from geometry_msgs.msg import TransformStamped, Transform

class World2RootTfMsgPublisher:
    def __init__(self):
        rospy.init_node('world_to_root_tf_msg_publisher')

        self.robot_ns = rospy.get_param('~robot_ns')
        self.pub = rospy.Publisher('world_to_root', TransformStamped, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform('world', self.robot_ns + '/root', rospy.Time.now(), rospy.Duration(0.5))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            self.pub.publish(trans)
            rate.sleep()

if __name__ == '__main__':
    node = World2RootTfMsgPublisher()
    rospy.spin()
