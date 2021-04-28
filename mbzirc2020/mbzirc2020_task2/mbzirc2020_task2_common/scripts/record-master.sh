#!/bin/sh

while getopts e: OPT
do
  case $OPT in
    e) id=$OPTARG
      ;;
  esac
done

echo $id

rosrun aerial_robot_base rosbag_control_data.sh /hydrus$id /hydrus$id/joints_ctrl /hydrus$id/joint_states /zed/odom /realsense1/odom/throttle /hydrus$id/realsense1/odom /hydrus$id/realsense2/odom/throttle /hydrus$id/realsense2/odom /hydrus$id/flight_state /hydrus$id/object_count /hydrus$id/task2_smach_server/smach/container_init /hydrus$id/task2_smach_server/smach/container_status /hydrus$id/task2_smach_server/smach/container_structure /hydrus$id/rs_d435/aligned_depth_to_color/camera_info /hydrus$id/rs_d435/aligned_depth_to_color/image_raw/compressedDepth /hydrus$id/rs_d435/color/camera_info /hydrus$id/rs_d435/color/image_rect_color/compressed /hydrus$id/rectangle_detection_color/target_object_color /hydrus$id/rectangle_detection_depth/target_object_depth /hydrus$id/task2_motion/nav_debug /hydrus$id/rosconsole_overlay_text/output /hydrus$id/channel_center/pos
