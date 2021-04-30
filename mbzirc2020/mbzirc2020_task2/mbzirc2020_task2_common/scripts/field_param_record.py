#!/usr/bin/env python

import rospy
from spinal.msg import Gps, Imu
import yaml

class GpsRecord:
    def __init__(self):
        rospy.init_node('gps_record')

        robot_ns = rospy.get_param('~robot_ns',default="hydrus")

        self.gps_sub = rospy.Subscriber(robot_ns + '/gps', Gps, self.gpsCallback)
        self.imu_sub = rospy.Subscriber(robot_ns + '/imu', Imu, self.imuCallback)
        self.location = [0, 0]
        self.yaw = 0

    def gpsCallback(self, msg):
        self.location = msg.location

    def imuCallback(self, msg):
        self.yaw = msg.angles[2]

    def execute(self):

        output = {}
        for i in range(2):
            get_key = raw_input()
            if get_key == "": #enter

                if i == 0:
                    print("%lf, %lf" % (self.location[0], self.location[1]))
                    output['global_lookdown_pos_gps'] = list(self.location)

                if i == 1:
                    print("%lf, %lf, %f" % (self.location[0], self.location[1], self.yaw))
                    output['global_place_channel_center_pos_gps'] = list(self.location)
                    output['global_place_channel_yaw'] = self.yaw

        with open('Field.yaml', 'w') as file:
            yaml.dump(output, file) #, default_flow_style=False)

        rospy.signal_shutdown("")

if __name__ == '__main__':
    node = GpsRecord()
    node.execute()
