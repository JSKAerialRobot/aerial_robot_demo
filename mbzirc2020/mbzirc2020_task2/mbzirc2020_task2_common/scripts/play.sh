#!/bin/bash

rosbag play $1 --clock --pause /rectangle_detection_color/target_object_color:=/rectangle_detection_color/target_object_color_orig /rectangle_detection_depth/target_object_depth:=/rectangle_detection_depth/target_object_depth_orig
