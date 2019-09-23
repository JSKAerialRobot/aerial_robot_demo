cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task2_simulation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  )

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc2020_task2_simulation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(mbzirc2020_task2_rubble_plugin src/mbzirc2020_task2_rubble_plugin.cpp)
target_link_libraries(mbzirc2020_task2_rubble_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
