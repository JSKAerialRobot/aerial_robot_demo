import numpy as np
import sys

def mDegLat(lat):
    lat_rad = lat * np.pi / 180.0

    return 111132.09 - 566.05 * np.cos(2.0 * lat_rad) + 1.20 * np.cos(4.0 * lat_rad) - 0.002 * np.cos(6.0 * lat_rad);

def mDegLon(lat):
    lat_rad = lat * np.pi / 180.0

    return 111415.13 * np.cos(lat_rad) - 94.55 * np.cos(3.0 * lat_rad) - 0.12 * np.cos(5.0 * lat_rad);

if __name__ == '__main__':
    args = sys.argv
    if len(args) != 5:
        print("invalid args num")
        exit(1)

    orig_lat = float(args[1])
    orig_lon = float(args[2])
    dst_lat = float(args[3])
    dst_lon = float(args[4])

    print(mDegLat(orig_lat) * (dst_lat - orig_lat), -mDegLon(orig_lat) * (dst_lon - orig_lon))
