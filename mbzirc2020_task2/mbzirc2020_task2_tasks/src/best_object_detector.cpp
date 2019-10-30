// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include </home/kuromiya/ros/aerial_robot_demo_ws/src/aerial_robot_demo/mbzirc2020_task2/mbzirc2020_task2_tasks/include/recognize.h>
#include </home/kuromiya/ros/aerial_robot_demo_ws/src/aerial_robot_demo/mbzirc2020_task2/mbzirc2020_task2_tasks/include/kmeans.cpp>
namespace mbzirc2020_task2_tasks
{

  void BestObjectDetectionWithContourSize::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("contour_area_size", contour_area_size_, 0.2);
    pnh_->param("contour_area_margin", contour_area_margin_, 0.01);
    pnh_->param("object_height", object_height_, 0.05);
    pnh_->param("frame_id", frame_id_, std::string("target_object"));
    pnh_->param("init_x", init_x, 0.0);
    pnh_->param("init_y", init_y, 0.0);
    pnh_->param("init_z", init_z, 0.0);
    always_subscribe_ = true;

    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/pos"), 1);
    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    xy_pub = advertise<geometry_msgs::Point>(*nh_, "xy_point", 1);
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);

    ros::Duration(1.0).sleep();
    sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", *nh_, ros::Duration(10.0));
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K(cam_info->K[0], cam_info->K[1], cam_info->K[2], cam_info->K[3], cam_info->K[4], cam_info->K[5], cam_info->K[6], cam_info->K[7], cam_info->K[8]);
    camera_K_inv_ = camera_K.inverse();
    real_size_scale_ = cam_info->K[0] * cam_info->K[4];
pp
    onInitPostProcess();
  }

  void BestObjectDetectionWithContourSize::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &BestObjectDetectionWithContourSize::imageCallback, this);
  }

  void BestObjectDetectionWithContourSize::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void BestObjectDetectionWithContourSize::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", msg->header.frame_id, msg->header.stamp);
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat src_image = cv_ptr->image;
    cv::Mat mask = cv::Mat(src_image != src_image);  // nan mask

    for (int i = 0; i < src_image.rows; i++){
      cv::Vec3b * ms = mask.ptr<cv::Vec3b>(i);
      cv::Vec3b * src = src_image.ptr<cv::Vec3b>(i);
      for (int j = 0; j < src_image.cols; j++){
        cv::Vec3b bgr = ms[j];
        if((mask[0] != 0) && (mask[1] != 0) && (mask[2] != 0)){
          src[j] = cv::Vec3b(255, 255, 255); // nan -> white
        }
      }
    }


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(src_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    double object_distance = cam_tf.getOrigin().z();
    double object_distance2 = object_distance * object_distance;
    double dist_from_center_min = 1e6;
    std::vector<std::vector<cv::Point>> target_contours;

    auto calc_position = [](std::vector<cv::Point> contour) {
      cv::Moments contour_moments = cv::moments(contour, true);
      tf2::Vector3 pos;
      pos.setX(contour_moments.m10 / contour_moments.m00);
      pos.setY(contour_moments.m01 / contour_moments.m00);
      return pos;
    };

    cv::Mat debug_image = cv::Mat::zeros(src_image.rows, src_image.cols, CV_8U);

    for(const auto& contour : contours) {
      double real_contour_area = cv::contourArea(contour) * object_distance2 / real_size_scale_;
      NODELET_DEBUG_STREAM("contour size" << real_contour_area);

      if(std::abs(real_contour_area - contour_area_size_) < contour_area_margin_) {
        target_contours.push_back(contour);
        }
      if (debug_view_) {
        cv::drawContours(debug_image, std::vector<std::vector<cv::Point> >(contours), -1, cv::Scalar(255, 255, 255), -1);
        image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, debug_image).toImageMsg());
      }
    }
  }
} //namespace aerial_robot_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc2020_task2_tasks::RedObjectDetectionWithHSVFilter, nodelet::Nodelet);

