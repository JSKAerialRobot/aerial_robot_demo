#pragma once

#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <aerial_robot_msgs/FlightNav.h>

#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>

namespace mbzirc2020_task2_tasks
{
  class RedObjectDetectionWithHSVFilter:public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    RedObjectDetectionWithHSVFilter():DiagnosticNodelet("Redobjectdetectionwithhsvfilter"){}

  protected:
    /* ros publisher */
    ros::Publisher target_pos_pub_;
    image_transport::Publisher image_pub_;
    ros::Publisher message_pub_;
    
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber message_sub_;
    
    boost::shared_ptr<image_transport::ImageTransport> it_;

    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    bool debug_view_;
    std::string frame_id_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_inv_;
    tf2::Matrix3x3 camera_K;

    double object_distance;
    double cam_angle = 0.0;
    std::string message = "start";

    int phase = 0;
    geometry_msgs::Vector3Stamped obj_pos_msg;
    
    tf2::Vector3 cam_target_xyz;

    void messageCallback(const std_msgs::String::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };
}
