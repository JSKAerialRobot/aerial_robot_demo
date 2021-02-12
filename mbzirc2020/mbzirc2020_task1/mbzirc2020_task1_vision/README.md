# Vision detection and tracking for the drone-ball target in MBZIRC Challenge1


## target: ball + drone

## detection mode
### 1. color filter based (default mode):
1. bounding box detection and tracking for drone-ball model based on SSD detector (see edgetpu_roscpp)
2. ball extraction based on color filter from the bounding box area
3. calculate the ball depth from the ball extraction and the drone width (bounding box width)

### 2. cascaded deep tracking:
1. bounding box detection and tracking for drone-ball model based on SSD detector (see edgetpu_roscpp)
2. do second detection based on another SSD dector inside the bound box to find the drone and ball separately.
3. calculate the ball depth from the ball extraction and the drone width (bounding box width)

## training:
- based on the training procedure in [edgetpu_roscpp](https://github.com/tongtybj/edgetpu_roscpp/blob/master/training/README.md):
  - guess you have created `${HOME}/object_learn/train_data/dataset`
- download dataset:
  ```
  $ rosrun mbzirc2020_task1_vision install_train_data.py
  ```
- copy `.tfrecord` files in following directories under `mbzirc2020_task1_vision/train/drone_and_ball` to `${HOME}/object_learn/train_data/dataset/train`:
  - for on-site testbed: `20191229_kashiwa_DJI450_mavic2pro_1080HD`, `20200103_kashwia_DJI450_hydrus_rsd435_720HD`, `20200121_kashwia_DJI450_hydrus_rsd435_720HD`, `20200122_kashwia_DJI450_hydrus_rsd435_720HD`, `20200124_kashwia_DJI450_hydrus_rsd435_720HD`, `20200130_kashwia_DJI450_hydrus_rsd435_720HD`, `20200204-outdoor-f450-net-hydrus-elp-fov60-1080HD`
  - for real compeition in ADNEC: `annotation-20200220`, `mbzirc-rehearsal-annotation-20200221`, `mbzirc-rehearsal-annotation-20200222`, `mbzirc-rehearsal-annotation-20200223`

- copy `mbzirc2020_task1_vision/train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD/tf_label_map.pbtxt` to `${HOME}/object_learn/train_data/dataset`

- only for training dataset of ADNEC real compeition (for better transfor learning): copy `mbzirc2020_task1_vision/train/drone_and_ball/train_from_original_ssd_5000_batch64_augment_20191229_to_2020204_624` to `${HOME}/object_learn/train_data/`

- modify the configuration in pipeline.config:
  - assume you have started docker for training (please follow [this README.md](https://github.com/tongtybj/edgetpu_roscpp/blob/master/training/README.md))
  - inside the docker, edit `/tensorflow/models/research/object_learn/train_data/ckpt/pipeline.config` with follown modification:
    - use smae dataset for train and val: `input_path: "/tensorflow/models/research/object_learn/train_data/dataset/train/*.tfrecord"` in scope of `eval_input_reader`
    - only for training dataset of ADNEC real competition (for better transfor learning): `fine_tune_checkpoint: "/tensorflow/models/research/object_learn/train_data/train_from_original_ssd_5000_batch64_augment_20191229_to_2020204_624/model.ckpt-5000"` in scope of `train_config`


### OPTION: cascaded deep tracking:

Under construction. Please contact Moju Zhao to prepare for the training script if you want to train this mode.


##  detection and tracking commands:

### color filter based
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info
```

### cascaded deep tracking

- use cacaded deep detection for ball and drone detection
```
$ roslaunch mbzirc2020_task1_vision drone_ball_tracking_by_detection.launch image_view:=true verbose:=false color_image_topic:=/rs_d435/color/image_rect_color camera_info_topic:=/rs_d435/color/camera_info cascaded_detection:=true
```

## test with rosbag:

### download test data:
```
$ rosrun mbzirc2020_task1_vision install_vision_test_data.py
```

### run command:
```
$ rqt_image_view /drone_ball_detection_and_tracking/detection_result_image
$ roscd mbzirc2020_task1_vision/test/data
$ rosbag play 2020-01-03-kashiwa-hydrus-rsd435-task1-detection-rawdata2-1.bag
```

