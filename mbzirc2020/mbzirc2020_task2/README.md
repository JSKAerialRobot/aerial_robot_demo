# MBZIRC 2020 Task2
## Task rule: please refer to [official website](https://www.mbzirc.com/challenge/2020)

![task2](../images/task2.png)

## Bringup

### real machine
```
$ roslaunch mbzirc2020_task2_common bringup.launch
```

### real machine including motion and recognition
```
$ roslaunch mbzirc2020_task2_common challenge.launch
```

### simulation (including motion and recognition)
Default
```
$ roslaunch mbzirc2020_task2_simulation simulation.launch
```

Light world
```
$ roslaunch mbzirc2020_task2_simulation simulation.launch world:=light
```


## Ground Vehicle Model

### real machine
```
$ roslaunch mbzirc2020_task2_common bringup.launch ground_mode:=True
```

### simulation
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False ground_mode:=True
```

## Cooperative Task

### real machine

In external PC
```
$ rossetlocal
$ rossetip
$ roslaunch mbzirc2020_task2_common coop_laptop_master_transport.launch
```

In the leader machine  
```
$ rossetmaster (external PC's IP)
$ rossetip
$ roslaunch mbzirc2020_task2_common coop_task_agent.launch robot_id:=1 leader:=true
```

In the follower machine  
```
$ rossetmaster (external PC's IP)
$ rossetip
$ roslaunch mbzirc2020_task2_common coop_task_agent.launch robot_id:=2 leader:=false
```

### simulation
```
$ roslaunch mbzirc2020_task2_common coop_laptop_master_transport.launch simulation:=true real_machine:=false headless:=false
```

### simulation indoor
```
$ roslaunch mbzirc2020_task2_common coop_laptop_master_transport.launch simulation:=true real_machine:=false headless:=false outdoor:=false
```



## Debug

```
$ roslaunch mbzirc2020_task2_common rosbag_replay.launch robot_ns:=[hydrus1 or hydrus2]
```

```
$ rosrun mbzirc2020_task2_common play.sh [robot_id (1 or 2)] [rosbag file]
```

- Then press space key to play rosbag.
