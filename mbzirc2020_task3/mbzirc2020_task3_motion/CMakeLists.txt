cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task3_motion)

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
#  LIBRARIES mbzirc2020_task3_motion
#  CATKIN_DEPENDS mbzirc2020_task3_common roscpp rospy
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Declare a C++ library
add_library(open_form_planner src/open_form_planner.cpp )
target_link_libraries(open_form_planner ${catkin_LIBRARIES})

add_executable(open_form_planner_node src/open_form_planner_node.cpp)
target_link_libraries(open_form_planner_node ${catkin_LIBRARIES} open_form_planner)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
