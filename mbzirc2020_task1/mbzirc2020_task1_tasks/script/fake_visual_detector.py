#!/usr/bin/env python
import rospy
import sys
import time
import math
import copy

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Bool

NOT_START = 0
NOT_FOUND = 1
FOUND = 2

class fakeVisualDetector:
    def init(self):
        rospy.init_node('fake_visual_detector', anonymous=True)
        rospy.sleep(5.0)

        self.__uav_object_offset_z = rospy.get_param('~uav_object_offset_z', 0.27)

        self.__object_find_dist_rate_thre = 2.0

        self.__visual_tracking_state = NOT_START
        self.__treasure_capture_flag = False

        self.__object_gt_odom = Odometry()
        self.__hydrus_odom = Odometry()
        self.__object_gt_odom.pose.pose.position.x = 10000.0
        self.__object_gt_odom.pose.pose.position.y = 10000.0
        self.__object_gt_odom.pose.pose.position.z = 10000.0
        self.__hydrus_odom.pose.pose.position.x = -10000.0
        self.__hydrus_odom.pose.pose.position.y = -10000.0
        self.__hydrus_odom.pose.pose.position.z = 10000.0

        self.__object_gt_odom_sub = rospy.Subscriber("/hawk/ground_truth/state", Odometry, self.__objectGTOdomCallback)
        self.__hydrus_odom_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.__hydrusOdomCallback)
        self.__hydrus_motion_init_flag_sub = rospy.Subscriber("/track/hydrus_motion_init_flag", Empty, self.__hydrusMotionInitCallback)
        self.__treasure_capture_flag_sub = rospy.Subscriber("/treasure/capture_flag", Bool, self.__treasureCaptureFlagCallback)
        self.__detected_object_odom_pub = rospy.Publisher('/vision/object_odom', Odometry, queue_size=1)
        rospy.sleep(1.0)

    def __objectGTOdomCallback(self, msg):
        self.__object_gt_odom = msg

        if self.__visual_tracking_state == NOT_START:
            return
        if self.__visual_tracking_state == NOT_FOUND:
            pos_delta = [self.__object_gt_odom.pose.pose.position.x - self.__hydrus_odom.pose.pose.position.x,
                         self.__object_gt_odom.pose.pose.position.y - self.__hydrus_odom.pose.pose.position.y,
                         self.__object_gt_odom.pose.pose.position.z - self.__hydrus_odom.pose.pose.position.z]
            dist = 0.0
            for i in range(0, 3):
                dist += math.pow(pos_delta[i], 2)
            dist = math.sqrt(dist)

            if dist < self.__object_find_dist_rate_thre * abs(pos_delta[2]):
                self.__visual_tracking_state = FOUND

        if self.__visual_tracking_state == FOUND:
            ## todo: add case for state change: FOUND -> NOT_FOUND
            self.__object_gt_odom.pose.pose.position.z -= self.__uav_object_offset_z
            self.__detected_object_odom_pub.publish(self.__object_gt_odom)

    def __hydrusOdomCallback(self, msg):
        self.__hydrus_odom = msg

    def __hydrusMotionInitCallback(self, msg):
        self.__visual_tracking_state = NOT_FOUND

    def __treasureCaptureFlagCallback(self, msg):
        if msg.data:
            self.__treasure_capture_flag = True
            self.__visual_tracking_state = NOT_FOUND
            time.sleep(3)
        else:
            self.__treasure_capture_flag = False
            self.__visual_tracking_state = FOUND

if __name__ == '__main__':
    try:
        ## test
        fake_visual_detector = fakeVisualDetector()
        fake_visual_detector.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
