#!/bin/sh

while getopts e: OPT
do
  case $OPT in
    e) id=$OPTARG
      ;;
  esac
done

echo $id

rosrun aerial_robot_base rosbag_control_data.sh /hydrus$id /hydrus$id/joints_ctrl /hydrus$id/joint_states /zed/odom /realsense1/odom/throttle /realsense1/odom /realsense2/odom/throttle /realsense2/odom /flight_state /object_count /task2_smach_server/smach/container_init /task2_smach_server/smach/container_status /task2_smach_server/smach/container_structure /rs_d435/aligned_depth_to_color/camera_info /rs_d435/aligned_depth_to_color/image_raw/compressedDepth /rs_d435/color/camera_info /rs_d435/color/image_rect_color/compressed /rectangle_detection_color/target_object_color /rectangle_detection_depth/target_object_depth /task2_motion/nav_debug /rosconsole_overlay_text/output /channel_center/pos