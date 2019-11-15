cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc2020_common
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
