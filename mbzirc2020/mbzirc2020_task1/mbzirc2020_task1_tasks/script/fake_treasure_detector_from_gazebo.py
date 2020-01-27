#!/usr/bin/env python
import rospy
import sys
import time
import math
import copy

from geometry_msgs.msg import PointStamped

class fakeVisualDetector:
    def init(self):
        rospy.init_node('fake_treasure_detector_from_gazebo', anonymous=True)
        self.__treasure_gazebo_point_freq = 1000
        self.__treasure_detection_freq = 25
        self.__treasure_detection_cnt = 0

        self.__treasure_point_from_gazebo_sub = rospy.Subscriber("/treasure/point_from_gazebo", PointStamped, self.__treasurePointFromGazeboCallback)
        self.__treasure_detected_point_pub = rospy.Publisher('/treasure/point_detected', PointStamped, queue_size=1)
        rospy.sleep(1.0)

    def __treasurePointFromGazeboCallback(self, msg):
        if self.__treasure_detection_cnt >= self.__treasure_gazebo_point_freq / self.__treasure_detection_freq:
            self.__treasure_detection_cnt = 0
        if self.__treasure_detection_cnt == 0:
            self.__treasure_detected_point_pub.publish(msg)
        self.__treasure_detection_cnt += 1

if __name__ == '__main__':
    try:
        ## test
        fake_visual_detector = fakeVisualDetector()
        fake_visual_detector.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
