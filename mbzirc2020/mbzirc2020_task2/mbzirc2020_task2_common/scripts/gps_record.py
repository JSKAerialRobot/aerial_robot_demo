#!/usr/bin/env python

import rospy
from spinal.msg import Gps
import pyperclip

class GpsRecord:
    def __init__(self):
        rospy.init_node('gps_record')

        self.sub = rospy.Subscriber('/gps', Gps, self.callback)
        self.location = None

    def callback(self, msg):
        self.location = msg.location

    def execute(self):

        while not rospy.is_shutdown():
            get_key = raw_input()

            if get_key == "": #enter
                print("%lf, %lf" % (self.location[0], self.location[1]))
                clip_string = "[" + str(self.location[0]) + ", " + str(self.location[1]) + "]"
                pyperclip.copy(clip_string)

if __name__ == '__main__':
    node = GpsRecord()
    node.execute()
