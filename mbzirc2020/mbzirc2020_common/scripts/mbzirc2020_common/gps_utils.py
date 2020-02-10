import numpy as np


def mDegLat(lat):
    lat_rad = lat * np.pi / 180.0
    return 111132.09 - 566.05 * np.cos(2.0 * lat_rad) + 1.20 * np.cos(4.0 * lat_rad) - 0.002 * np.cos(6.0 * lat_rad)

def mDegLon(lat):
    lat_rad = lat * np.pi / 180.0
    return 111415.13 * np.cos(lat_rad) - 94.55 * np.cos(3.0 * lat_rad) - 0.12 * np.cos(5.0 * lat_rad)

def gps2xy(orig_gps, dst_gps):
    return mDegLat(orig_gps[0]) * (dst_gps[0] - orig_gps[0]), -mDegLon(orig_gps[0]) * (dst_gps[1] - orig_gps[1])


def getGpsLocation(orig_gps, x, y):
    lat_diff = x / mDegLat(orig_gps[0])
    lon_diff = -y / mDegLon(orig_gps[0])

    return orig_gps[0] + lat_diff, orig_gps[1] + lon_diff
