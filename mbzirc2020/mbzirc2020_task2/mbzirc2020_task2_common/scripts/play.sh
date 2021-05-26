#!/bin/bash

echo ID=$1

NS=/hydrus$1

echo namespace=$NS

shift

rosbag play $@ --clock --pause $NS/rectangle_detection_color/target_object_color:=$NS/rectangle_detection_color/target_object_color_orig $NS/rectangle_detection_depth/target_object_depth:=$NS/rectangle_detection_depth/target_object_depth_orig $NS/kf/alt1/data:=$NS/kf/alt1/data_orig $NS/kf/gps1/data:=$NS/kf/gps1/data_orig $NS/kf/imu1/data:=$NS/kf/imu1/data_orig $NS/kf/vo1/data:=$NS/kf/vo1/data_orig $NS/kf/plane_detection1/data:=$NS/kf/plane_detection1/data_orig
