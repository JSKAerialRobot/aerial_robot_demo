#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform
import tf2_ros
import tf.transformations as tft
import ros_numpy

class GroundStationTfPublisher:
    def __init__(self):
        rospy.init_node('ground_station_tf_publisher')

        self.cog_odom_sub = rospy.Subscriber('/uav/cog/odom', Odometry, self.cogOdomCallback)
        self.cog2baselink_sub = rospy.Subscriber('/cog2baselink', TransformStamped, self.cog2baselinkCallback)

        self.br = tf2_ros.TransformBroadcaster()

    def cogOdomCallback(self, msg):
        t = Transform()
        t.translation.x = msg.pose.pose.position.x
        t.translation.y = msg.pose.pose.position.y
        t.translation.z = msg.pose.pose.position.z
        t.rotation = msg.pose.pose.orientation
        t_np = ros_numpy.numpify(t)
        t_np = tft.inverse_matrix(t_np)

        ts = TransformStamped()
        ts.header.stamp = msg.header.stamp
        ts.header.frame_id = 'cog'
        ts.child_frame_id = 'world'
        ts.transform = ros_numpy.msgify(Transform, t_np)

        self.br.sendTransform(ts)

    def cog2baselinkCallback(self, msg):
        t = Transform()
        t = msg.transform
        t_np = ros_numpy.numpify(t)
        t_np = tft.inverse_matrix(t_np)

        ts = TransformStamped()
        ts.header.stamp = msg.header.stamp
        ts.header.frame_id = 'fc'
        ts.child_frame_id = 'cog'
        ts.transform = ros_numpy.msgify(Transform, t_np)
        self.br.sendTransform(ts)


if __name__ == '__main__':
    node = GroundStationTfPublisher()
    rospy.spin()
