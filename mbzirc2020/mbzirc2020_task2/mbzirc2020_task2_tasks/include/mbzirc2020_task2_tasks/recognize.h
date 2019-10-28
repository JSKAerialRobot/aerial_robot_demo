#pragma once

#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_utils/geo/polygon.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
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
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <chrono>
#include <thread>

enum Phase
  {
   Phase0,
   Phase1,
   Phase2,
   Phase3
  };

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
    image_transport::Publisher image_depth_pub_;
    ros::Publisher xy_pub;
    ros::Publisher area_pub;
    ros::Publisher marker_pub;
    ros::Publisher angle_pub;
    ros::Publisher working_phase_pub;
    ros::Publisher object_marker_pub;
    ros::Publisher target_pos_pub;    
    ros::Publisher jointstate_pub;
    
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_depth_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber plane_sub_;
    ros::Subscriber working_phase_sub_;
    ros::Subscriber uav_odom_sub_;

    boost::shared_ptr<image_transport::ImageTransport> it_;

    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    double init_x, init_y, init_z;
    bool debug_view_;
    std::string frame_id_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_inv_;
    tf2::Matrix3x3 camera_K;
    jsk_recognition_utils::CameraDepthSensor camdep;

    double object_distance;
    std::vector<std::vector<int>> planes_img;
    std::vector<jsk_recognition_utils::Vertices> vertice_highest;
    cv::Mat depth_img;

    int working_phase;
    int working_phase_py;
    std_msgs::Int32 working_phase_msg;

    double hydrus_angle = 0.0;
    double cam_angle = 0.0;

    std::vector<std::vector<double>> target_pos_cnd;
    std::vector<double> target_angle_cnd;
    std::vector<int> target_count;
    
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;
    double target_angle_ = 0.0;

    tf2::Vector3 cam_target_xyz;
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imagedepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void planeCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    void workingphaseCallback(const std_msgs::Int32 msg);
    void uavodomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void openjoints();
    
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };
}
