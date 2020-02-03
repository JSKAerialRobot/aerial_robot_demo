### drone and ball detection

#### parameters
- model_file: choose detection model
- coarse_detection_score_threshold: default is `0`.
- refined_detection_score_threshold: default is `0.71`
- tracking_score_threshold: default is `0.7`


#### general commands:
- use both color filter and bounding box width
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false  model_file:=`rospack find mbzirc2020_task1_tasks`/models/drone_detection_edgetpu_20200130_509.tflite color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info
```

- only use color filter
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false  model_file:=`rospack find mbzirc2020_task1_tasks`/models/drone_detection_edgetpu_20200130_509.tflite color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info bbox_valid_bound_margin:=10000
```

- only use bounding box width
```
$ roslaunch mbzirc2020_task1_tasks drone_ball_tracking_by_detection.launch  image_view:=true verbose:=false  model_file:=`rospack find mbzirc2020_task1_tasks`/models/drone_detection_edgetpu_20200130_509.tflite color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info ball_far_depth_outlier_threshold:=-1 ball_close_depth_outlier_threshold:=-1
```