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

- run mbzirc_arena wolrd in gazebo:
```
$ roslaunch mbzirc2020_task1_common mbzirc_arena_1.launch
```

- run simulation of task1:
```
$ roslaunch mbzirc2020_task1_tasks jsk_mbzirc_task_1.launch
```

- enable hector motors in gazebo:
```
$ rosservice call /hawk/enable_motors "enable: true"
```

## task 1 standard mode

- run simulation of task1:
```
$ roslaunch mbzirc2020_task1_tasks jsk_mbzirc_task_1.launch
```

- hector moves in 8 shape:
```
$ rosrun mbzirc2020_task1_common guard_uav_routine_flight.py
```

## task 1 cheat mode

- run simulation of task1:
```
$ roslaunch mbzirc2020_task1_tasks jsk_mbzirc_task_1.launch
```

- run cheat mode of task1:
```
$ rosrun mbzirc2020_task1_tasks task_1_cheat.py
```
