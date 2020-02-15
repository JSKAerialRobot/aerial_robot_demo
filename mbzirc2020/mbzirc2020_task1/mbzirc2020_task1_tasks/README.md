

### command to perform task1:
```
$ roslaunch mbzirc2020_task1_tasks challenge.launch
```
####options:

- ground_station_ip: the IP address of ground station pc.

### drone and ball detection

#### parameters
- model_file: choose detection model
- coarse_detection_score_threshold: default is `0`.
- refined_detection_score_threshold: default is `0.71`
- tracking_score_threshold: default is `0.7`


#### general commands:
- use both color filter and bounding box width
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info cascaded_detection:=false
```

- only use color filter
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info cascaded_detection:=false bbox_valid_bound_margin:=10000 
```

- only use bounding box width
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info cascaded_detection:=false ball_far_depth_outlier_threshold:=-1 ball_close_depth_outlier_threshold:=-1
```

- use cacaded deep detection for ball and drone detection
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info cascaded_detection:=true
```
