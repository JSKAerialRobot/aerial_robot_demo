#!/bin/bash

rosbag play $@ --clock --pause /hydrus/rectangle_detection_color/target_object_color:=/hydrus/rectangle_detection_color/target_object_color_orig /hydrus/rectangle_detection_depth/target_object_depth:=/hydrus/rectangle_detection_depth/target_object_depth_orig hydrus/kf/alt1/data:=/hydrus/kf/alt1/data_orig hydrus/kf/gps1/data:=hydrus/kf/gps1/data_orig hydrus/kf/imu1/data:=hydrus/kf/imu1/data_orig hydrus/kf/vo1/data:=hydrus/kf/vo1/data_orig /hydrus/kf/plane_detection1/data:=/hydrus/kf/plane_detection1/data_orig
