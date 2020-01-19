#!/bin/sh

rosrun aerial_robot_base rosbag_control_data.sh /hydrusx/joints_ctrl /hydrusx/joint_states /zed/odom /realsense1/odom/throttle /realsense1/odom /realsense2/odom/throttle /realsense2/odom /flight_state /object_count /rectangle_detection_depth/debug_image/compressed /rs_d435/aligned_depth_to_color/image_raw /task2_smach_server/smach/container_init /task2_smach_server/smach/container_status /task2_smach_server/smach/container_structure
