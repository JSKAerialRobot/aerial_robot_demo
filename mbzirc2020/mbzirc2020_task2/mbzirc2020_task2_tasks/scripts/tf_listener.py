#!/usr/bin/env python

import numpy as np

import rospy
import tf
from tf.transformations import quaternion_matrix

from mbzirc2020_task2_common.hydrus_cooperation_interface import HydrusInterface

if __name__=='__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    robot_ns = 'hydrus'

    hydrus = HydrusInterface(robot_ns=robot_ns)

    rate = rospy.Rate(1)
    rospy.loginfo('start tf')
    while not rospy.is_shutdown():
        print('#####')
        try:
            (trans, rot) = listener.lookupTransform(robot_ns+'/cog',robot_ns+'/ball_joint12_left', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        force = hydrus.calc_ave_force(side='left')

        #a^p = a^R^b * b^p
        force_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(force))

        rate.sleep()
    



