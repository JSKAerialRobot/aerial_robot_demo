#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import UInt8
import tf.transformations
import tf2_ros

class DummyObjectTfPublisher:
    def __init__(self):
        rospy.init_node('dummy_object_tf_publisher')
        self.sub = rospy.Subscriber('/object_count', UInt8, self.objectCountCallback)
        self.global_object_pos = rospy.get_param('~global_object_pos')
        self.global_object_z = rospy.get_param('~global_object_z')
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.object_count = 0

    def objectCountCallback(self, msg):
        self.object_count = msg.data

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.object_count < len(self.global_object_pos):
                object_pos = self.global_object_pos[self.object_count]
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'world'
                t.child_frame_id = 'target_object'
                t.transform.translation.x = object_pos[0]
                t.transform.translation.y = object_pos[1]
                t.transform.translation.z = self.global_object_z
                q = tf.transformations.quaternion_from_euler(0, 0, object_pos[2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.broadcaster.sendTransform(t)
            rate.sleep()

if __name__ == '__main__':
    node = DummyObjectTfPublisher()
    node.execute()
    rospy.spin()


