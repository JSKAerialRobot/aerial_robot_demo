#!/bin/bash

rosrun hydrus rosbag_control_data.sh  /rs_d435/color/image_rect_color /rs_d435/color/camera_info /usb_cam/image_raw /usb_cam/camera_info  --split --duration=30
