cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task2_tasks)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  smach
  smach_ros
  mbzirc2020_common)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc2020_task2_tasks
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
