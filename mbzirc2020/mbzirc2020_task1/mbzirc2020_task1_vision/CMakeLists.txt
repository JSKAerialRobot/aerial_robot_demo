cmake_minimum_required(VERSION 2.8.3)
project(mbzirc2020_task1_vision)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  edgetpu_roscpp
  dynamic_reconfigure
  message_filters
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

catkin_package(
  INCLUDE_DIRS include
  )

# download detection models
add_custom_target(install_detection_models ALL
  COMMAND ${PROJECT_SOURCE_DIR}/script/install_detection_models.py)

# following command takes long time and size. Recommand to run these python script from commandline with a desktop
#add_custom_target(install_train_dataset ALL
#  COMMAND ${PROJECT_SOURCE_DIR}/script/install_train_data.py)
#add_custom_target(install_test_data ALL
#  COMMAND ${PROJECT_SOURCE_DIR}/script/install_vision_test_data.py)


add_library(drone_ball_tracking_detection SHARED
  src/color_filter_based_tracking_detection.cpp
  src/cascaded_deep_tracking_detection.cpp)
target_link_libraries(drone_ball_tracking_detection ${catkin_LIBRARIES} ${Boost_LIBRARIES})
