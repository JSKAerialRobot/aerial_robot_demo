#!/usr/bin/env python

'''
usage:
$ rosrun mbzirc2020_task1_common yaml_to_camera_info_publisher.py --filename `rospack find mbzirc2020_task1_common`/config/camera/elp_1080P_fov60.yaml
'''

import rospy
import argparse
import yaml
from sensor_msgs.msg import CameraInfo

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-f', '--filename', dest='filename', action="store",
        help='the name of tfrecord file', default='*', type=str)

    args = parser.parse_args()
    filename = args.filename

    print(filename)
    # Parse yaml file

    # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]


    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()
