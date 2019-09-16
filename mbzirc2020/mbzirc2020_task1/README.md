## MBZIRC 2020 Task 1 of JSK Lab

## how to compile (after cloning)

```
cd <catkin_ws>
wstool merge -t src src/aerial_robot_demo/mbzirc2020_task1/mbzirc_task1.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to use

- launch simulation of task1:
```
$ roslaunch mbzirc2020_task1_tasks jsk_mbzirc_task_1.launch
```

- launch tracking module of task1:
```
$ roslaunch mbzirc2020_task1_tasks task_1_tracking.launch
```

- run tracking command interface of task1:
```
$ rosrun mbzirc2020_task1_tasks task_command_interface.py
```


## task 1 cheat mode

- run simulation of task1:
```
$ roslaunch mbzirc2020_task1_tasks jsk_mbzirc_task_1.launch start:=false
```

- run cheat mode of task1:
```
$ rosrun mbzirc2020_task1_tasks task_1_cheat.py
```

## tips

- manually control hector in gazebo:
```
$ roslaunch mbzirc2020_task1_common mbzirc_arena_1.launch start:=false
$ rosservice call /hawk/enable_motors "enable: true"
```