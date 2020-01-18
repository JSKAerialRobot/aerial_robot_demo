#!/usr/bin/env python
import rospy

import math
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('ground_plane_in_sensor_frame')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()

    ground_frame = rospy.get_param('~ground_frame', 'world')
    sensor_frame = rospy.get_param('~sensor_frame', 'sensor_frame')

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            print sensor_frame
            trans = tfBuffer.lookup_transform(ground_frame, sensor_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        trans.child_frame_id = "ground_plane_in_sensor_frame"
        euler = tf.transformations.euler_from_quaternion((trans.transform.rotation.x,
                                                          trans.transform.rotation.y,
                                                          trans.transform.rotation.z,
                                                          trans.transform.rotation.w))
        q = tf.transformations.quaternion_from_euler(0, 0, euler[2])

        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        br.sendTransform(trans)

        rate.sleep()
