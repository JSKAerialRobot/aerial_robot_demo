#!/usr/bin/env python

import numpy as np
import sys
from gps2xy import *
import matplotlib.pyplot as plt
import rosbag

def getXY(orig_lat, orig_lon, dst_lat, dst_lon):
    return (mDegLat(orig_lat) * (dst_lat - orig_lat), -mDegLon(orig_lat) * (dst_lon - orig_lon))

if __name__ == '__main__':
    args = sys.argv
    bag_name = str(args[1])
    start_from = int(args[2])
    max_count = int(args[3])
    topic_name = "/gps"
    if len(args) > 4:
        topic_name = str(args[4])

    x = []
    y = []
    first_run = True
    orig_lat, orig_lon = 0, 0
    count = 0

    for topic, msg, t in rosbag.Bag(bag_name):
        if topic == topic_name:
            if first_run:
                orig_lat = msg.location[0]
                orig_lon = msg.location[1]
                first_run = False

            count += 1
            if count < start_from:
                continue

            lat = msg.location[0]
            lon = msg.location[1]
            x_, y_ = getXY(orig_lat, orig_lon, lat, lon)
            x.append(x_)
            y.append(y_)

            if count > max_count:
                break

    x = np.array(x)
    x_mean = np.mean(x)
    x = x - x_mean

    y = np.array(y)
    y_mean = np.mean(y)
    y = y - y_mean

    plt.plot(range(len(x)), x, label='x')
    plt.plot(range(len(y)), y, label='y')

    plt.legend(loc='best')

    plt.show()
