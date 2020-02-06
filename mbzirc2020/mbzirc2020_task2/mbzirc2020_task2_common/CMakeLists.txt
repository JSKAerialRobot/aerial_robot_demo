cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task2_common)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  hydrus
  mbzirc2020_task2_simulation
  gazebo_ros
  timed_roslaunch
  message_generation
  sensor_msgs
  jsk_topic_tools
  jsk_network_tools
  geometry_msgs
  nav_msgs
  jsk_rviz_plugins
  std_msgs
  )

add_message_files(
  FILES
  FC2OCSData.msg
  OCS2FCData.msg
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  tf2_msgs
  jsk_rviz_plugins
  geometry_msgs
  nav_msgs
  std_msgs
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
