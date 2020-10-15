#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print("ERROR: please give the rosbag file name")
    exit(1)

if len(sys.argv) > 2:
    print("ERROR: too many args")
    exit(1)

bag_name = sys.argv[1]

start_time = None

#imu_log = open("imu.txt", mode="w")
odom_log = open("odom.txt", mode="w")
alt_log = open("alt.txt", mode="w")
# alt_log = open("alt.txt", mode="w")
# plane_log = open("plane.txt", mode="w")
# vo_log = open("vo.txt", mode="w")


for topic, msg, t in rosbag.Bag(bag_name):
    if start_time is None:
        start_time = t

    if topic == "/hydrus/uav/baselink/odom":
        data = msg.pose.pose.position.z
        stamp = (t - start_time).to_sec()
        odom_log.write(str(stamp) + ', ' + str(data) + '\n')

    # if topic == "/hydrus/kf/alt1/data":
    #     data = msg.range
    #     stamp = (t - start_time).to_sec()
    #     alt_log.write(str(stamp) + ', ' + str(data) + '\n')

    if topic == "/hydrus/kf/alt1/data":
        data = msg.states[0].state[0].x
        stamp = (t - start_time).to_sec()
        alt_log.write(str(stamp) + ', ' + str(data) + '\n')

    # if topic == "/hydrus/kf/plane_detection1/data":
    #     data = msg.states[0].state[0].x
    #     stamp = (t - start_time).to_sec()
    #     plane_log.write(str(stamp) + ', ' + str(data) + '\n')

    # if topic == "hydrus/kf/vo1/data":
    #     data = msg.states[2].state[0].x
    #     stamp = (t - start_time).to_sec()
    #     vo_log.write(str(stamp) + ', ' + str(data) + '\n')

# imu_log.close()
# alt_log.close()
# plane_log.close()
# vo_log.close()

alt_log.close()
odom_log.close()
