# Repository for MBZIRC

## how to compile

### Beginner:
```
cd <catkin_ws>
wstool merge -t src src/aerial_robot_demo/travis.rosinstall # install packages about transformable aerial robot first!
wstool merge -t src src/aerial_robot_demo/mbzirc2020/mbzirc_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

### Developer of transformable aerial robot:
```
cd <catkin_ws>
wstool merge -t src src/aerial_robot_demo/mbzirc2020/mbzirc_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```


## Introduction of important dependencies


### transformable robot platform
- aeiral_robot:
   - url: https://github.com/JSKAerialRobot/aerial_robot
   - version: `458f1a2` # temporal version. Stable version will come soon

### vision
- aeiral_robot_recognition:
   - purpose: general aerial vision
   - url: https://github.com/JSKAerialRobot/aerial_robot_recognition
   - version: `master`
- edgetpu_roscpp:
   - purpose: deep object detection and tracking for task1
   - url: http://github.com/tongtybj/edgetpu_roscpp.git 
   - version: `master`

### other dependencies:

Please check `mbzirc_${ROS_DISTRO}.rosinstall`, which is necessary to run when you build all packages
