# MBZIRC 2020 Task3
## Task rule: please refer to [official website](https://www.mbzirc.com/challenge/2020)

![task1](../images/task1.png)

## Bringup

### real machine
**COMING SOON**

### simulation
#### bringup:
```
$ roslaunch mbzirc2020_task1_tasks motion.launch  real_machine:=false simulation:=true headless:=false

#### run tracking command interface of task1:
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

