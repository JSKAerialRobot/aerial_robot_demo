// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#include <mbzirc2020_task1_tasks/drone_ball_tracking_detection.h>

namespace edgetpu_roscpp
{
  void DroneBallTrackingDetection::onInit()
  {
    SingleObjectDeepTrackingDetection::onInit();

    pnh_->param("subscribe_depth_image", subscribe_depth_image_, false);
    pnh_->param("ball_candidate_area_rate", ball_candidate_area_rate_, 0.5); // rate of the lower part of the bounding box
    pnh_->param("ball_real_radius", ball_real_radius_, 0.075);
    pnh_->param("approx_contour_rate", approx_contour_rate_, 0.1);

    color_filter_reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<opencv_apps::HLSColorFilterConfig> >(*pnh_);
    typename dynamic_reconfigure::Server<opencv_apps::HLSColorFilterConfig>::CallbackType f = boost::bind(&DroneBallTrackingDetection::colorFilterReconfigureCallback, this, _1, _2);
    color_filter_reconfigure_server_->setCallback(f);

    if(subscribe_depth_image_)
      {
        color_depth_images_sync_ = std::make_unique<message_filters::Synchronizer<ColorDepthSyncPolicy> >(ColorDepthSyncPolicy(2), snyc_color_image_sub_, snyc_depth_image_sub_);
        color_depth_images_sync_->registerCallback(boost::bind(&DroneBallTrackingDetection::colorDepthSyncCallback, this,  _1, _2));
      }

    if (image_view_) debug_image_pub_ = advertiseImage(*pnh_, "color_filtered_image", 1);
    ball_marker_pub_ = advertise<visualization_msgs::Marker>(*nh_, "ball_marker", 1);
    ball_pos_pub_ = advertise<geometry_msgs::PointStamped>(*nh_, "ball_pos", 1);
  }

  void DroneBallTrackingDetection::subscribe()
  {
    if(subscribe_depth_image_)
      {
        snyc_color_image_sub_.subscribe(*nh_, "color_image", 1);
        snyc_depth_image_sub_.subscribe(*nh_, "depth_image", 1);
      }
    else
      {
        SingleObjectDeepTrackingDetection::subscribe();
      }

    cam_info_sub_ = nh_->subscribe("color_camera_info", 1, &DroneBallTrackingDetection::cameraInfoCallback, this);
  }

  void DroneBallTrackingDetection::unsubscribe()
  {
    if(subscribe_depth_image_)
      {
        snyc_color_image_sub_.unsubscribe();
        snyc_depth_image_sub_.unsubscribe();
      }
    else
      {
        SingleObjectDeepTrackingDetection::unsubscribe();
      }
  }

  void DroneBallTrackingDetection::colorDepthSyncCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
  {
    imageCallback(color_msg);

#if 0
    if(ball_pixel_radius_ > 0)
      {
        /* compare with depth with */
        cv::Mat depth_img = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        ROS_INFO("the depth of ball center in depth image: %f vs %f", depth_img.at<unsigned char>(ball_pixel_center_.y, ball_pixel_center_.x)/1000.0f, ball_pos_.z());
      }
#endif
  }

  void DroneBallTrackingDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K(msg->K[0], msg->K[1], msg->K[2],
                            msg->K[3], msg->K[4], msg->K[5],
                            msg->K[6], msg->K[7], msg->K[8]);
    camera_K_inv_ = camera_K.inverse();
    f_dash_ = sqrt(msg->K[0] * msg->K[4]);

    cam_info_sub_.shutdown();
  }

  void DroneBallTrackingDetection::detection_tracking_process(cv::Mat& src_img)
  {
    /* detect and track the drone */
    SingleObjectDeepTrackingDetection::detection_tracking_process(src_img);

    if(!detected_)
      {
        ball_pixel_radius_ = -1;
        return;
      }

    double start_t = ros::Time::now().toSec();

    /* further crop from the bonding box to find the ball */
    double detected_bbox_width = best_detection_candidate_.corners.xmax - best_detection_candidate_.corners.xmin;
    double detected_bbox_height = best_detection_candidate_.corners.ymax - best_detection_candidate_.corners.ymin;

    /* assumption: ball is in the lower part of the bounding box since it is hung by drone */
    cv::Mat ball_search_img;
    //ball_search_img = src_img(cv::Rect(best_detection_candidate_.corners.xmin, best_detection_candidate_.corners.ymax - detected_bbox_height * ball_candidate_area_rate_, best_detection_candidate_.corners.xmax - best_detection_candidate_.corners.xmin, detected_bbox_height * ball_candidate_area_rate_));
    auto ball_search_area = best_detection_candidate_.corners;
    ball_search_area.ymin = best_detection_candidate_.corners.ymax - detected_bbox_height * ball_candidate_area_rate_;

    //std::cout << "[" << ball_search_area.xmin << ", " << ball_search_area.ymin << ", " << ball_search_area.xmax << ", " << ball_search_area.ymax << "] -> ";
    expandedBoundingImage(src_img, ball_search_area, 1.2, ball_search_img, ball_search_area); // expand
    //std::cout << "[" << ball_search_area.xmin << ", " << ball_search_area.ymin << ", " << ball_search_area.xmax << ", " << ball_search_area.ymax << "]" << std::endl;

    /* color filter */

    colorFilter(ball_search_img, color_filtered_img_);

    /* find contours */
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(color_filtered_img_, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    if(contours.size() == 0) return;

    std::vector<cv::Point> target_contour;
    double target_contour_area = 0;
    for(const auto& contour : contours)
      {
        double contour_area = cv::contourArea(contour);

        if(contour_area >  target_contour_area)
          {
            target_contour_area = contour_area;
            target_contour = contour;
          }
      }

    /* estiamte the ball radius and center from contour */
#if 0
    /* https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html */
    std::vector<cv::Point> approx_contour;
    double epsilon = approx_contour_rate_ * cv::arcLength(target_contour, true);
    cv::approxPolyDP(target_contour, approx_contour, epsilon, true);
    target_contour = approx_contour;
#endif

#if 0 // draw the contour in the src img
    for (auto& point: target_contour)
      {
        point.x += ball_search_area.xmin;
        point.y += ball_search_area.ymin;
      }
    std::vector<std::vector<cv::Point> > contours_for_drawing(1, target_contour);
    cv::drawContours(src_img, contours_for_drawing, 0, cv::Scalar(0, 255, 255), 2);
#endif

    cv::minEnclosingCircle(target_contour, ball_pixel_center_, ball_pixel_radius_);

#if 0   // debug, check every pixel (HLS) of the ball
    cv::Mat temp_image;
    cv::cvtColor(ball_search_img, temp_image, cv::COLOR_RGB2HLS);
    for(int j = -ball_pixel_radius_; j < ball_pixel_radius_; j++)
      {
        for(int i = -ball_pixel_radius_; i < ball_pixel_radius_; i++)
          {
            if(i*i + j * j > ball_pixel_radius_ * ball_pixel_radius_)
              std::cout <<  std::setw(3) << "---";
            else
              std::cout <<  std::setw(3) << 2 * (int)temp_image.at<cv::Vec3b>(ball_pixel_center_.y + j, ball_pixel_center_.x + i)[0];
            std::cout << ", ";
          }
        std::cout << std::endl;
      }
    for(int j = -ball_pixel_radius_; j < ball_pixel_radius_; j++)
      {
        for(int i = -ball_pixel_radius_; i < ball_pixel_radius_; i++)
          {
            if(i*i + j * j > ball_pixel_radius_ * ball_pixel_radius_)
              std::cout <<  std::setw(3) << "---";
            else
              std::cout <<  std::setw(3) << (int)temp_image.at<cv::Vec3b>(ball_pixel_center_.y + j, ball_pixel_center_.x + i)[1];
            std::cout << ", ";
          }
        std::cout << std::endl;
      }
    for(int j = -ball_pixel_radius_; j < ball_pixel_radius_; j++)
      {
        for(int i = -ball_pixel_radius_; i < ball_pixel_radius_; i++)
          {
            if(i*i + j * j > ball_pixel_radius_ * ball_pixel_radius_)
              std::cout <<  std::setw(3) << "---";
            else
              std::cout <<  std::setw(3) << (int)temp_image.at<cv::Vec3b>(ball_pixel_center_.y + j, ball_pixel_center_.x + i)[2];
            std::cout << ", ";
          }
        std::cout << std::endl;
      }

    throw std::runtime_error("test");
#endif

    ball_pixel_center_.x += ball_search_area.xmin;
    ball_pixel_center_.y += ball_search_area.ymin;
    ROS_INFO("ball pixel center: [%f, %f], radius: %f", ball_pixel_center_.x, ball_pixel_center_.y, ball_pixel_radius_);
    cv::circle(src_img, ball_pixel_center_, (int)ball_pixel_radius_, cv::Scalar(255, 0, 0), 2);

    /* 3d position of ball */
    if(f_dash_ == 0) return;
    double ball_depth = f_dash_ * ball_real_radius_ / ball_pixel_radius_;
    ball_pos_ = camera_K_inv_ * tf2::Vector3(ball_pixel_center_.x, ball_pixel_center_.y, 1.0) * ball_depth;

    ROS_INFO("ball position: [%f, %f, %f], depth: %f", ball_pos_.x(),
             ball_pos_.y(), ball_pos_.z(), ball_depth);
    //ROS_WARN("ball detection: %f", ros::Time::now().toSec() - start_t);
  }

  void DroneBallTrackingDetection::colorFilter(const cv::Mat& input_image, cv::Mat& output_image)
  {
    cv::Mat cvt_image;
    //cv::cvtColor(input_image, cvt_image, cv::COLOR_RGB2CVT);
    cv::cvtColor(input_image, cvt_image, cv::COLOR_RGB2HLS);
    if (lower_color_range_[0] < upper_color_range_[0])
      {
        cv::inRange(cvt_image, lower_color_range_, upper_color_range_, output_image);
      }
    else
      {
        cv::Scalar lower_color_range_0 = cv::Scalar(0, l_min_, s_min_, 0);
        cv::Scalar upper_color_range_0 = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
        cv::Scalar lower_color_range_360 = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
        cv::Scalar upper_color_range_360 = cv::Scalar(360 / 2, l_max_, s_max_, 0);
        cv::Mat output_image_0, output_image_360;
        cv::inRange(cvt_image, lower_color_range_0, upper_color_range_0, output_image_0);
        cv::inRange(cvt_image, lower_color_range_360, upper_color_range_360, output_image_360);
        output_image = output_image_0 | output_image_360;
      }
  }

  void DroneBallTrackingDetection::colorFilterReconfigureCallback(opencv_apps::HLSColorFilterConfig& config, uint32_t level)
  {
    color_filter_config_ = config;
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    l_max_ = config.l_limit_max;
    l_min_ = config.l_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;

    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (l_max_ < l_min_)
      std::swap(l_max_, l_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
  }

  void DroneBallTrackingDetection::publish(const std_msgs::Header& msg_header, const cv::Mat& src_img)
  {
    SingleObjectDeepTrackingDetection::publish(msg_header, src_img);


    if(ball_pixel_radius_ < 0) return;

    /* publish the position of ball */
    geometry_msgs::PointStamped ball_pos_msg;
    ball_pos_msg.header = msg_header;
    tf2::toMsg(ball_pos_, ball_pos_msg.point);
    ball_pos_pub_.publish(ball_pos_msg);

    visualization_msgs::Marker marker_msg;
    marker_msg.header = msg_header;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.pose.position = ball_pos_msg.point;
    marker_msg.pose.orientation.w = 1;
    marker_msg.scale.x = ball_real_radius_ * 2;
    marker_msg.scale.y = marker_msg.scale.x;
    marker_msg.scale.z = marker_msg.scale.x;
    marker_msg.color.r = 1;
    marker_msg.color.g = 1;
    marker_msg.color.a = 1;
    ball_marker_pub_.publish(marker_msg);

    geometry_msgs::TransformStamped ball_tf;
    ball_tf.header = msg_header;
    ball_tf.child_frame_id = std::string("ball");
    ball_tf.transform.translation = tf2::toMsg(ball_pos_);
    ball_tf.transform.rotation.w = 1.0;
    tf_br_.sendTransform(ball_tf);

    if (image_view_)
      debug_image_pub_.publish(cv_bridge::CvImage(msg_header,  sensor_msgs::image_encodings::MONO8, color_filtered_img_).toImageMsg());
  }

} //namespace edgetpu_roscpp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (edgetpu_roscpp::DroneBallTrackingDetection, nodelet::Nodelet);
