cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task2_common)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  hydrus
  mbzirc2020_task2_simulation
  gazebo_ros
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc2020_task2_common
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
