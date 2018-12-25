cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_common)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp std_msgs geometry_msgs)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS thread random)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

# build plugins
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(mbzirc_gazebo_panel_plugin src/mbzirc_gazebo_panel_plugin.cpp)
target_link_libraries(mbzirc_gazebo_panel_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(mbzirc_gazebo_uav_collision_rule_plugin src/mbzirc_gazebo_uav_collision_rule_plugin.cpp)
target_link_libraries(mbzirc_gazebo_uav_collision_rule_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(mbzirc_gazebo_treasure_plugin src/mbzirc_gazebo_treasure_plugin.cpp)
target_link_libraries(mbzirc_gazebo_treasure_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# build executable program
add_executable(uav_teleop_keyboard src/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS
  mbzirc_gazebo_panel_plugin
  mbzirc_gazebo_uav_collision_rule_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY launch gazebo_model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

