#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Transform
import tf2_ros
import tf.transformations as tft

class GroundStationTfPublisher:
    def __init__(self):
        rospy.init_node('ground_station_tf_publisher')

        self.world_to_root_sub = rospy.Subscriber('world_to_root', TransformStamped, self.worldToRootCallback)
        self.br = tf2_ros.TransformBroadcaster()

    def worldToRootCallback(self, msg):
        self.br.sendTransform(msg)

if __name__ == '__main__':
    node = GroundStationTfPublisher()
    rospy.spin()
