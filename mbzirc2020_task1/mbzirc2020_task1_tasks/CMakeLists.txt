cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task1_tasks)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_generation
  message_runtime
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3 REQUIRED)
# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
message(STATUS "CMAKE_BUILD_TYPE:               ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linear_kalman_filter trajectory_predictor
  CATKIN_DEPENDS roscpp rospy
  DEPENDS EIGEN3
  )

add_library(linear_kalman_filter src/LinearKalmanFilter.cpp)
target_link_libraries(linear_kalman_filter ${catkin_LIBRARIES})

add_library(trajectory_predictor src/TrajectoryPredictor.cpp)
target_link_libraries(trajectory_predictor ${catkin_LIBRARIES} linear_kalman_filter)

add_executable(tracking_trajectory_estimator_node src/trajectory_predictor_node.cpp)
target_link_libraries(tracking_trajectory_estimator_node trajectory_predictor ${catkin_LIBRARIES})
