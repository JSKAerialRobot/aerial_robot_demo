cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task2_tasks)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  jsk_topic_tools
  nav_msgs
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
  nodelet
  image_view
  jsk_perception
  aerial_robot_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge geometry_msgs jsk_topic_tools nav_msgs roscpp rospy tf2_ros nodelet jsk_perception
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

macro(arp_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

arp_add_nodelet(src/recognize.cpp "mbzirc2020_task2_tasks/RedObjectDetectionWithHSVFilter" "red_object_detection_with_hsv_filter")

add_library(${PROJECT_NAME} SHARED ${mbzirc2020_task2_tasks_nodelet_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()
