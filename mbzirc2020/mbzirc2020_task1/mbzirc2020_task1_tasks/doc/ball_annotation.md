## Download the annotated images
```
$ rosrun mbzirc2020_task1_tasks install_train_data.py
```


## Preparation for annoatated image 
### export images from rosbag
Example:
```
$ roslaunch edgetpu_roscpp export_images_from_rosbag.launch rosbag_file:=/home/chou/Downloads/2020-01-30-15-56-52_1.bag image_topic:=/rs_d435/color/image_rect_color duration:=2 sec_per_frame:=0.1 filename_prefix:=2020-01-30-15-56-52_1_image
```

### prepare for the cropped training dataset
1. use python3 (i.e. 3.5 for 16.04) in virtualenv in host pc:
```
$ sudo apt install python-virtualenv
$ cd ~ && mkdir -p python-venv && cd python-venv
$ virtualenv -p python3.5 venv-py3-5
$ source venv-py3-5/bin/activate
```

2. install tensorflow and pillow in venv
```
$ pip install --upgrade tensorflow Pillow
```

3. extract cropped image from pre-annotated image dataset
```
$ cd ./script
$ ./cropped_image_from_annotated_tfrecord.py --path=`rospack find mbzirc2020_task1_tasks`/train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD/class1_drone --filename=DJI_0052.MP4#t=5 --output=/tmp --show=True # single image 
```

