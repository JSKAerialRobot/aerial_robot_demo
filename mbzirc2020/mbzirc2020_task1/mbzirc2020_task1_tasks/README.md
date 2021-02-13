# Autonomous Aerial Interception

## Real Machine

```
$ roslaunch mbzirc2020_task1_tasks challenge.launch
```
### options:
- ground_station_ip: the IP address of ground station pc.

## Simulation

### bringup:
```
$ roslaunch mbzirc2020_task1_tasks motion.launch  real_machine:=false simulation:=true headless:=false
```
### run tracking command interface of task1:
```
$ rosrun mbzirc2020_task1_tasks task_command_interface.py
```

### cheat mode
#### bringup:
```
$ roslaunch mbzirc2020_task1_tasks motion.launch  real_machine:=false simulation:=true headless:=false start:=false
```

#### run cheat mode of task1:
```
$ rosrun mbzirc2020_task1_tasks task_1_cheat.py
```

### other tips

- manually control hector in gazebo:
```
$ roslaunch mbzirc2020_task1_tasks motion.launch  real_machine:=false simulation:=true headless:=false start:=false
$ rosservice call /hawk/enable_motors "enable: true"
```




## Component Evaluation (Developer)

### target visual detection and tracking 

#### parameters
- model_file: choose detection model
- coarse_detection_score_threshold: default is `0`.
- refined_detection_score_threshold: default is `0.71`
- tracking_score_threshold: default is `0.7`


#### general commands:
- use both color filter and bounding box width
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/usb_cam/image_raw camera_info_topic:=/usb_cam/camera_info
```

- only use color filter
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/usb_cam/image_raw camera_info_topic:=/usb_cam/camera_info bbox_valid_bound_margin:=10000 
```

- only use bounding box width
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/usb_cam/image_raw camera_info_topic:=/usb_cam/camera_info ball_far_depth_outlier_threshold:=-1 ball_close_depth_outlier_threshold:=-1
```

- use cacaded deep detection for ball and drone detection
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/usb_cam/image_raw camera_info_topic:=/usb_cam/camera_info cascaded_detection:=true
```

### Trajectory Estimation by RANSAC
