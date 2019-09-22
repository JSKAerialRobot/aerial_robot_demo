cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task3_tasks)

find_package(catkin REQUIRED COMPONENTS
  mbzirc2020_task3_common
  roscpp
  rospy
  std_srvs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS mbzirc2020_task3_common roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



