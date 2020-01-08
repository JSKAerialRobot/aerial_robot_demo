cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task1_tasks)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  edgetpu_roscpp
  message_filters
  message_generation
  message_runtime
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3 REQUIRED)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS thread)

# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

add_message_files(
  FILES
  PrimitiveParams.msg
  )
generate_messages(DEPENDENCIES std_msgs)

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
  ${Boost_INCLUDE_DIR}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linear_kalman_filter trajectory_predictor
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS EIGEN3
  )

# download detection models
add_custom_target(install_detection_models ALL
  COMMAND ${PROJECT_SOURCE_DIR}/script/install_detection_models.py)

add_library(motion_single_primitive src/MotionSinglePrimitive.cpp)
target_link_libraries(motion_single_primitive ${catkin_LIBRARIES})

add_library(linear_kalman_filter src/LinearKalmanFilter.cpp)
target_link_libraries(linear_kalman_filter ${catkin_LIBRARIES})

add_library(trajectory_predictor src/TrajectoryPredictor.cpp)
target_link_libraries(trajectory_predictor ${catkin_LIBRARIES} linear_kalman_filter)

add_library(trajectory_tracker src/TrajectoryTracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES} trajectory_predictor motion_single_primitive)

add_library(ransac_line_fitting src/RansacLineFitting.cpp)
target_link_libraries(ransac_line_fitting ${catkin_LIBRARIES})

add_executable(trajectory_predictor_node src/trajectory_predictor_node.cpp)
target_link_libraries(trajectory_predictor_node trajectory_predictor ${catkin_LIBRARIES})

add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
target_link_libraries(trajectory_tracker_node trajectory_tracker ${catkin_LIBRARIES})

# Detection
add_library(drone_ball_tracking_detection SHARED src/drone_ball_tracking_detection.cpp)
target_link_libraries(drone_ball_tracking_detection ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(ransac_line_fitting_node src/ransac_line_fitting_node.cpp)
target_link_libraries(ransac_line_fitting_node ransac_line_fitting ${catkin_LIBRARIES})
