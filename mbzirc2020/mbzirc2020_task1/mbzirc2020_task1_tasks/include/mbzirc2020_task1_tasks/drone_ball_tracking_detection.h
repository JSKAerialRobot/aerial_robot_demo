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

#pragma once

#include <dynamic_reconfigure/server.h>
#include <edgetpu_roscpp/single_object_tracking_by_deep_detection.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv_apps/HSVColorFilterConfig.h>
#include <opencv_apps/HLSColorFilterConfig.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

namespace edgetpu_roscpp
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ColorDepthSyncPolicy;

  class DroneBallTrackingDetection: public SingleObjectDeepTrackingDetection
  {
  public:
    DroneBallTrackingDetection():
      SingleObjectDeepTrackingDetection(),
      f_dash_(0),
      ball_depth_(-1)
    {}

  protected:
    ros::Subscriber cam_info_sub_;
    image_transport::Publisher debug_image_pub_;
    ros::Publisher ball_marker_pub_;
    ros::Publisher ball_pos_pub_;
    tf2_ros::TransformBroadcaster tf_br_;

    /* consider the depth image */
    message_filters::Subscriber<sensor_msgs::Image> snyc_color_image_sub_, snyc_depth_image_sub_;
    std::unique_ptr<message_filters::Synchronizer<ColorDepthSyncPolicy> > color_depth_images_sync_;

    /* parameter */
    bool subscribe_depth_image_;

    /* bounding box based depth detection */
    int bbox_valid_bound_margin_;
    double drone_real_width_; // this is very naive parameter, since the width will change acoording to the view angle

    /* color filter of ball detection */
    double ball_real_radius_;
    double ball_candidate_area_rate_;
    int h_min_;
    int h_max_;
    int s_min_;
    int s_max_;
    int l_min_;
    int l_max_;
    double approx_contour_rate_;
    cv::Mat color_filtered_img_;
    cv::Point2f ball_pixel_center_;
    tf2::Vector3 ball_pos_;

    /* overrall evaluation */
    double ball_depth_;
    double ball_depth_lpf_gain_;
    double ball_far_depth_outlier_threshold_;
    double ball_close_depth_outlier_threshold_;
    double far_depth_;
    double close_depth_;

    /* camera info */
    double f_dash_;
    tf2::Matrix3x3 camera_K_inv_;

    /* HLS is better than HSV */
    opencv_apps::HLSColorFilterConfig color_filter_config_;
    boost::shared_ptr<dynamic_reconfigure::Server<opencv_apps::HLSColorFilterConfig> > color_filter_reconfigure_server_;

    cv::Scalar lower_color_range_;
    cv::Scalar upper_color_range_;

    void colorDepthSyncCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    bool widthBasedDetection(cv::Mat& src_img, double& drone_depth);
    bool colorFilterBallDetection(cv::Mat& src_img, double& ball_depth);

    void onInit() override;
    void subscribe() override;
    void unsubscribe() override;

    void detection_tracking_process(cv::Mat& src_img) override;
    void publish(const std_msgs::Header& msg_header, const cv::Mat& src_img) override;

    void colorFilterReconfigureCallback(opencv_apps::HLSColorFilterConfig& config, uint32_t level);
    void colorFilter(const cv::Mat& input_image, cv::Mat& output_image);


  };
};
