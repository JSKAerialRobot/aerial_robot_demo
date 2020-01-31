#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["frame_id"]
    camera_info_msg.width = calib_data["width"]
    camera_info_msg.height = calib_data["height"]
    camera_info_msg.K = calib_data["K"]
    camera_info_msg.D = calib_data["D"]
    camera_info_msg.R = calib_data["R"]
    camera_info_msg.P = calib_data["P"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)

    filename = rospy.get_param('~camera_info_file')

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)
    publisher = rospy.Publisher("~camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()
