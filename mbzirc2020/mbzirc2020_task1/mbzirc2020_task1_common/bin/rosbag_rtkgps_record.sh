#!/bin/bash

rosrun hydrus rosbag_control_data.sh hydrus  hydrus/rtk_gps/fix hydrus/rtk_gps/fix_velocity hydrus/rtk_gps/navpvt hydrus/rtk_gps/navrelposned hydrus/rtk_gps/rel_pos rtk_gps/rtcm /hydrus/kf/gps2/data

