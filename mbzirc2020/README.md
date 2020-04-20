# Repository for MBZIRC

## dependency

- aeiral_robot:
   - url: https://github.com/tongtybj/aerial_robot
   - version: `tag = 1.1.1`

- aeiral_robot_recognition:
   - url: https://github.com/tongtybj/aerial_robot_recognition
   - version: `master`
   

## how to compile

```
cd <catkin_ws>
wstool merge -t src src/aerial_robot_demo/mbzirc2020/mbzirc_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```
