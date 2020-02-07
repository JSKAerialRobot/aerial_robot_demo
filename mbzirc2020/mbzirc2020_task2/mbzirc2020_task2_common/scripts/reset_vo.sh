#!/bin/bash

rosnode kill /rs_d435/color_rectify_color /rs_d435/points_xyzrgb_hw_registered /rs_d435/realsense2_camera /rs_d435/realsense2_camera_manager
sleep 8
rosservice call /realsense1/reset
sleep 1
roslaunch realsense2_camera rs_rgbd.launch camera:=rs_d435 depth_fps:=15 infra_fps:=15 color_fps:=15 > /dev/null
