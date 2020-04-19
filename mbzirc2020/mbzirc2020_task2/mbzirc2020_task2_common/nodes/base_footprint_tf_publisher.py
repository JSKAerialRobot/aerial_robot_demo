#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform
import tf2_ros
import tf.transformations as tft
import ros_numpy

class BaseFootprintTfPublisher:
    def __init__(self):
        rospy.init_node('base_footprint_tf_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.br = tf2_ros.TransformBroadcaster()

    def execute(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform('world', 'fc', rospy.Time(), rospy.Duration(0.5))
                trans.transform.translation.z = 0.0
                trans.child_frame_id = "base_footprint"
                self.br.sendTransform(trans)

            except:
                rospy.logerr("tf listen failed")

            r.sleep()


if __name__ == '__main__':
    node = BaseFootprintTfPublisher()
    node.execute()
    rospy.spin()
