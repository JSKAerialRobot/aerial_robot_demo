#!/bin/sh

rosrun aerial_robot_base rosbag_control_data.sh /realsense1/odom/throttle /realsense1/odom /realsense2/odom/throttle /realsense2/odom /lepton_camera/color/image /lepton_camera/color/camera_info /lepton_camera/temperature/camera_info /lepton_camera/temperature/image /lepton_camera/threshold/camera_info /lepton_camera/threshold/image /target_object/pos /task3_start /estimator/sensor_plugin/vo2/data /estimator/sensor_plugin/vo1/data /estimator/sensor_plugin/imu/data
