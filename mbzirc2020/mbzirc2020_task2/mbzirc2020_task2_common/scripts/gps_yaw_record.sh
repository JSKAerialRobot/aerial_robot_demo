#!/usr/bin/env python

import rospy
from spinal.msg import Gps, Imu
import pyperclip

class GpsRecord:
    def __init__(self):
        rospy.init_node('gps_record')

        self.gps_sub = rospy.Subscriber('/gps', Gps, self.gpsCallback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imuCallback)
        self.location = [0, 0]
        self.yaw = 0

    def gpsCallback(self, msg):
        self.location = msg.location

    def imuCallback(self, msg):
        self.yaw = msg.angles[2]

    def execute(self):

        clip_string = ""
        for i in range(2):
            get_key = raw_input()
            if get_key == "": #enter

                if i == 0:
                    print("%lf, %lf" % (self.location[0], self.location[1]))
                    clip_string = "global_lookdown_pos_gps: [" + str(self.location[0]) + ", " + str(self.location[1]) + "]"

                if i == 1:
                    print("%lf, %lf, %f" % (self.location[0], self.location[1], self.yaw))
                    clip_string += "\n" + "global_place_channel_center_pos_gps: [" + str(self.location[0]) + ", " + str(self.location[1]) + "]" + "\n" + "global_place_channel_yaw: " + str(self.yaw)

        pyperclip.copy(clip_string)
        rospy.signal_shutdown("")

if __name__ == '__main__':
    node = GpsRecord()
    node.execute()
