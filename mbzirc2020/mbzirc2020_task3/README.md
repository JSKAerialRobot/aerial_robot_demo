# MBZIRC 2020 Task3
## Task rule: please refer to [official website](https://www.mbzirc.com/challenge/2020)

![task3](../images/task3.png)

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


### if you want to use the horizontal VIO (realsense T265) in indoor enviroment, please:

   1. attach the horizontal VIO sensor module
   2. do following command:
   ```
   $ roslaunch mbzirc2020_task3_tasks motion.launch horizontal_vio:= true
   ```

*note*: horizontal vio sensor provides bettwer performance in indoor enviroment, but do not use in outdoor environment!
