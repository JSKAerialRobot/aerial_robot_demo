# aerial mode on simulation

$ roslaunch hydrus_ground_transportation bringup.launch

$ rostopic pub -1 /task_start std_msgs/Empty "{}"

# aerial mode on real machine

You cannot do this because [you cannot build aerial_robot_perception](https://github.com/tongtybj/aerial_robot_recoginition/issues/5)).

# ground mode on simulation

$ roslaunch hydrus_ground_transportation bringup.launch ground:=True

$ rostopic pub -1 /task_start std_msgs/Empty "{}"

If the hydrus cannot recognize a red object, it can be that the parameter of hsi-filter is not adequate. You can adjust by using rqt_reconfigure and saving the setting to config/hsi_color_filter_for_sim.yaml.

# ground mode on real machine
--- on hydrus(euclid) side ---

$ roslaunch hydrus_ground_transportation bringup.launch simulation:=False real_machine:=True headless:=True ground:=True estimate_mode:=1

$ rosservice call /attitude~~

--- on PC side ---

$ rossetrobot (hydrus ip address)

$ rossetip

$ rviz
subscribe hsi_output topic on rviz

$ roslaunch aerial_robot_pratice hsi_filter.launch

$ rosrun aerial_robot_base key_board.launch
this is for halting hydrus when emergency by typing 'h'

--- on hydrus(euclid) side ---

$ rostopic pub -1 /task_start std_msgs/Empty "{}"


