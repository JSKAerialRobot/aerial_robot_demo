#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import copy
import numpy as np

class DepthImageFormatConverter:
    def __init__(self):
        rospy.init_node("depth_image_format_converter")

        self.image_sub = rospy.Subscriber('~input', Image, self.callback)
        self.image_pub = rospy.Publisher('~image', Image, queue_size = 1)
        self.bridge = cv_bridge.CvBridge()

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        dst_img = copy.copy(cv_image)
        cv2.normalize(cv_image, dst_img, 0, 255, cv2.NORM_MINMAX)
        dst_img = np.array(dst_img, dtype=np.uint8)

        output_msg = self.bridge.cv2_to_imgmsg(dst_img, "mono8")
        output_msg.header = msg.header
        self.image_pub.publish(output_msg)


if __name__ == '__main__':
    node = DepthImageFormatConverter()
    rospy.spin()
