# MBZIRC 2020 Task2
## Task rule: please refer to [official website](https://www.mbzirc.com/challenge/2020)

![task3](../images/task2.png)

## Bringup

### real machine
TODO


### simulation
Default
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False
```

Light world
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False worldtype:=light
```

Single object world
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False worldtype:=single_object
```
