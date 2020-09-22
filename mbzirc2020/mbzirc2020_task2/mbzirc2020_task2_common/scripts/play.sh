#!/bin/bash

rosbag play $@ --clock --pause /hydrus/rectangle_detection_color/target_object_color:=/hydrus/rectangle_detection_color/target_object_color_orig /hydrus/rectangle_detection_depth/target_object_depth:=/hydrus/rectangle_detection_depth/target_object_depth_orig
