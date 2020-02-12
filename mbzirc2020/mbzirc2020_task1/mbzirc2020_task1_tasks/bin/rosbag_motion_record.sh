#!/bin/sh

rosrun hydrus rosbag_control_data.sh /realsense1/odom/throttle /realsense1/odom /rs_d435/color/image_rect_color /rs_d435/color/camera_info /usb_cam/image_raw /usb_cam/camera_info  /reactive_motion/state /reactive_motion/target /reactive_motion/cog_point /treasure/line_fitting_marker /treasure/point_detected /treasure/points_fitting_markers /ball_pos /drone_ball_detection_and_tracking/detection_result_image/compressed  --split --duration=30
