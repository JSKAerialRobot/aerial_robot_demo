#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print("ERROR: please give the rosbag file name")
    exit(1)

if len(sys.argv) > 2:
    print("ERROR: too many args")
    exit(1)

bag_name = sys.argv[1]
bag_name_tmp = bag_name.split('.')[0]
output_bag_name = bag_name_tmp + '_converted.bag'

topic_map = {'/aerial_robot_control_four_axis' : '/hydrus/four_axes/command',
             '/baro' : '/hydrus/baro',
             '/battery_voltage_status' : '/hydrus/battery_voltage_status',
             '/channel_center/pos' : '/hydrus/channel_center/pos',
             '/control_terms' : '/hydrus/rpy/pid',
             '/desire_coordinate' : '/hydrus/desire_coordinate',
             '/distance' : '/hydrus/leddar_one/range',
             '/estimator/sensor_plugin/gps/data' : '/hydrus/kf/gps1/data',
             '/estimator/sensor_plugin/imu/data' : '/hydrus/kf/imu1/data',
             '/estimator/sensor_plugin/vo/data' : '/hydrus/kf/vo1/data',
             '/flight_config_ack' : '/hydrus/flight_config_ack',
             '/flight_config_cmd' : '/hydrus/flight_config_cmd',
             '/flight_state' : '/hydrus/flight_state',
             '/four_axis_gain' : '/hydrus/debug/four_axes/gain',
             '/gps' : '/hydrus/gps',
             '/hydrusx/joint_states' : '/hydrus/joint_states',
             '/hydrusx/joints_ctrl' : '/hydrus/joints_ctrl',
             '/imu' : '/hydrus/imu',
             '/joy' : '/hydrus/joy',
             '/motor_info' : '/hydrus/motor_info',
             '/motor_pwms' : '/hydrus/motor_pwms',
             '/p_matrix_pseudo_inverse_inertia' : '/hydrus/p_matrix_pseudo_inverse_inertia',
             '/realsense1/odom' : '/hydrus/realsense1/odom',
             '/realsense1/odom/throttle' : '/hydrus/realsense1/odom/throttle',
             '/rectangle_detection_color/target_object_color' : '/hydrus/rectangle_detection_color/target_object_color',
             '/rosconsole_overlay_text/output' : '/hydrus/rosconsole_overlay_text/output',
             '/rosout' : '/rosout',
             '/rosout_agg' : '/rosout_agg',
             '/rpy_gain' : '/hydrus/rpy/gain',
             '/rs_d435/aligned_depth_to_color/camera_info' : '/hydrus/rs_d435/aligned_depth_to_color/camera_info',
             '/rs_d435/aligned_depth_to_color/image_raw/compressedDepth' : '/hydrus/rs_d435/aligned_depth_to_color/image_raw/compressedDepth',
             '/rs_d435/color/camera_info' : '/hydrus/rs_d435/color/camera_info',
             '/rs_d435/color/image_rect_color/compressed' : '/hydrus/rs_d435/color/image_rect_color/compressed',
             '/servo/states' : '/hydrus/servo/states',
             '/servo/target_states' : '/hydrus/servo/target_states',
             '/task2_motion/nav_debug' : '/hydrus/task2_motion/nav_debug',
             '/task2_smach_server/smach/container_status' : '/hydrus/task2_smach_server/smach/container_status',
             '/task2_smach_server/smach/container_structure' : '/hydrus/task2_smach_server/smach/container_structure',
             '/tf' : '/tf',
             '/uav/baselink/odom' : '/hydrus/uav/baselink/odom',
             '/uav/cog/odom' : '/hydrus/uav/cog/odom',
             '/uav/full_state' : '/hydrus/uav/full_state',
             '/uav/nav' : '/hydrus/uav/nav',
             '/uav_info' : '/hydrus/uav_info',
             '/uav_power' : '/hydrus/uav_power'}

with rosbag.Bag(output_bag_name, 'w') as bag_out:
    for topic, msg, t in rosbag.Bag(bag_name):
        if topic_map.get(topic):
            bag_out.write(topic_map.get(topic), msg, t)
