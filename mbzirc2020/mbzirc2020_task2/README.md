## Installation

```
cd <catkin_ws>
wstool merge -t src src/aerial_robot_demo/mbzirc2020_task2/mbzirc_task2.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```


## How to run simulation
Default
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False
```

Light world
```
$ roslaunch mbzirc2020_task2_common bringup.launch real_machine:=False simulation:=True headless:=False worldtype:=light
```
