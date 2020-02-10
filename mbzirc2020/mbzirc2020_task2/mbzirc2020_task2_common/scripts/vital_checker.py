#!/usr/bin/env python

import rospy
from spinal.msg import Gps
from sensor_msgs.msg import Image, Range, Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

RED = '\033[31m'
GREEN = '\033[32m'

class VitalChecker:
    def __init__(self):
        rospy.init_node('vital_cheker')

        self.gps_sub_ = rospy.Subscriber('/gps', Gps, self.gpsCallback)
        self.gps_timestamp_ = None

        self.d435_img_sub_ = rospy.Subscriber('/rs_d435/color/image_rect_color', Image, self.imgCallback)
        self.img_timestamp_ = None

        self.t265_odom_sub_ = rospy.Subscriber('/realsense1/odom/throttle', Odometry, self.odomCallback)
        self.odom_timestamp_ = None

        self.batt_sub_ = rospy.Subscriber('/battery_voltage_status', Float32, self.battCallback)
        self.batt_vol_ = None

        self.leddar_sub_ = rospy.Subscriber('/distance', Range, self.leddarCallback)
        self.leddar_timestamp_ = None

        self.joy_sub_ = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.joy_timestamp_ = None

    def gpsCallback(self, msg):
        self.gps_timestamp_ = rospy.Time.now()

    def imgCallback(self, msg):
        self.img_timestamp_ = rospy.Time.now()

    def odomCallback(self, msg):
        self.odom_timestamp_ = rospy.Time.now()

    def battCallback(self, msg):
        self.batt_vol_ = msg.data

    def leddarCallback(self, msg):
        self.leddar_timestamp_ = rospy.Time.now()

    def joyCallback(self, msg):
        self.joy_timestamp_ = rospy.Time.now()

    def checkDevice(self, name, timestamp):
        now = rospy.Time.now()

        if (timestamp is not None) and (now - timestamp).to_sec() < 0.5:
            print(GREEN + name + " OK")
        else:
            print(RED + name + " NG")

    def check(self):
        now = rospy.Time.now()

        self.checkDevice("GPS", self.gps_timestamp_)
        self.checkDevice("IMAGE", self.img_timestamp_)
        self.checkDevice("ODOM", self.odom_timestamp_)
        self.checkDevice("LEDDAR", self.leddar_timestamp_)
        self.checkDevice("JOY", self.joy_timestamp_)

        print("BATT: " + str(self.batt_vol_))
        print("")


if __name__ == '__main__':
    node = VitalChecker()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        node.check()
        r.sleep()
