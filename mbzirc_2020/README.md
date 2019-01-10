## MBZIRC 2020 of JSK Lab

## how to run program

- run mbzirc_arena wolrd in gazebo:
```
$ roslaunch mbzirc_common mbzirc_arena_1.launch
```

- run simulation of task1:
```
$ roslaunch mbzirc_tasks jsk_mbzirc_task_1.launch
```

- enable hector motors in gazebo:
```
$ rosservice call /hawk/enable_motors "enable: true"
```

## task 1 cheat mode

- run simulation of task1:
```
$ roslaunch mbzirc_tasks jsk_mbzirc_task_1.launch
```

- run cheat mode of task1:
```
$ rosrun mbzirc_tasks task_1_cheat.py
```
