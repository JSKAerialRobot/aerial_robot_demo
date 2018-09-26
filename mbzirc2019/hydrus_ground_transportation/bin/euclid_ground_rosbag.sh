#!/bin/sh

rosbag record /camera/color/image_raw/compressed /aerial_robot/ground_pose /aerial_robot/pose /baro /battery_voltage_status /camera_velocity /distance /gps /realsense/odom /realsense/slam/status /imu /joy /motor_pwms /servo/states /uav/nav /flight_config_ack /tf /tf_static /camera/depth_registered/points_color/throttle
