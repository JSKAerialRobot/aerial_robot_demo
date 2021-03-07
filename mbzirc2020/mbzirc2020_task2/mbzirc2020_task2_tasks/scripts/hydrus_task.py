#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty

def TaskStartCallback(_):

    robot_ns = rospy.get_param('/hydrus_task/robot_ns','hydrus')

    start_pub = rospy.Publisher(robot_ns + '/teleop_command/start', Empty, queue_size=10)
    takeoff_pub = rospy.Publisher(robot_ns + '/teleop_command/takeoff', Empty, queue_size=10)
    rospy.sleep(1)

    #start
    start_pub.publish(Empty())
    rospy.sleep(1)

    #takeoff
    takeoff_pub.publish(Empty())



if __name__ == '__main__':
    rospy.init_node('hydrus_task')

    taskstart_sub = rospy.Subscriber('/task_start',Empty,TaskStartCallback)

    rospy.spin()



