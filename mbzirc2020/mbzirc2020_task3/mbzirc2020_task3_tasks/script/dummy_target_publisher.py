#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Vector3Stamped

def talker():
    pub = rospy.Publisher('/target_object/pos', Vector3Stamped, queue_size=10)
    rospy.init_node('dummy_target_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    v = Vector3()
    v.x = rospy.get_param('~dummy_x', 1)
    v.y = rospy.get_param('~dummy_y', 1)
    v.z = rospy.get_param('~dummy_z', 0)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        msg = Vector3Stamped()
        msg.header = h
        msg.vector = v
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

