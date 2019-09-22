# MBZIRC 2020 Task3
## Task rule: please refer to [official website](https://www.mbzirc.com/challenge/2020)

![task3](images/task3.png)

## Bringup

### real machine
```
$ roslaunch mbzirc2020_task3_tasks motion.launch
```

### simulation
```
$ roslaunch mbzirc2020_task3_tasks motion.launch  real_machine:=false simulation:=true headless:=false
```

## Start motion
```
$ rostopic pub -1 /task3_start std_msgs/Empty "{}"
```

## Other tips

### horizontal VIO (realsense t265)
if you want to use the horizontaon VIO, please:
1. set `horizontal_vio = 1` in `mbzirc2020_task3_common/robots/hydrus.urdf.xacro`.
2. add `horizontal_vio := true` to bringup:
   ```
   $ roslaunch mbzirc2020_task3_tasks motion.launch horizontal_vio := true
   ```

*note*: horizontal vio sensor provides bettwer performance in indoor enviroment, but do not use in outdoor environment!
