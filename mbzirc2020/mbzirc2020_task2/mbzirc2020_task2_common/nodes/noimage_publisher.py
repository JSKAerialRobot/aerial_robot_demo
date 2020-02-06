#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo
import cv2
import os.path
import cv_bridge

class NoImagePublisher:

    def __init__(self):
        rospy.init_node('no_image_publisher')

        self.image_sub = rospy.Subscriber('~input', Image, self.imageCallback)
        self.cam_info_sub = rospy.Subscriber('~camera_info', CameraInfo, self.cameraInfoCallback)
        self.pub = rospy.Publisher('~image', Image, queue_size = 1)
        self.bridge = cv_bridge.CvBridge()

        self.update_rate = rospy.get_param('~update_rate', 30)
        self.image_shape = None
        self.image = None

    def cameraInfoCallback(self, msg):
        self.image_shape = (msg.width, msg.height)
        self.cam_info_sub.unregister()

    def imageCallback(self, msg):
        self.image = msg

    def execute(self):
        rospack = rospkg.RosPack()
        no_image_data = cv2.imread(os.path.join(rospack.get_path('mbzirc2020_task2_common'), 'image/no_image.png'))
        while self.image_shape is None:
            pass
        no_image_data = cv2.resize(no_image_data, self.image_shape)

        r = rospy.Rate(self.update_rate)

        try:
            while not rospy.is_shutdown():
                if (self.image is None) or ((rospy.Time.now() - self.image.header.stamp).to_sec() > 1.0):
                    self.pub.publish(self.bridge.cv2_to_imgmsg(no_image_data, "bgr8"))
                else:
                    self.pub.publish(self.image)
                    r.sleep()
        except:
            pass

if __name__ == '__main__':
    node = NoImagePublisher()

    node.execute()
