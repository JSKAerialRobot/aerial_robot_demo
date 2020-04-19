#!/bin/sh

rosrun aerial_robot_base rosbag_control_data.sh /realsense1/odom/throttle /realsense1/odom /rs_d435/color/image_rect_color /rs_d435/color/camera_info /usb_cam/image_raw  --split --duration=30
