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

#include <mbzirc2020_task1_tasks/vision/cascaded_deep_tracking_detection.h>

namespace
{
  int ball_cnt = 0;
  int drone_cnt = 0;
  int detected_cnt = 0;

};

namespace edgetpu_roscpp
{
  void CascadedDeepBallTracking::onInit()
  {
    ColorFilterBallTracking::onInit();
    std::string model_file;
    pnh_->param("separate_model_file", model_file, std::string(""));
    pnh_->param("separate_model_detection_score_threshold", separate_model_detection_score_threshold_, 0.5);
    seperated_model_detection_engine_ = boost::make_shared<coral::DetectionEngine>(model_file);
  }

  void CascadedDeepBallTracking::subscribe()
  {
    ColorFilterBallTracking::subscribe();
  }

  void CascadedDeepBallTracking::unsubscribe()
  {
    ColorFilterBallTracking::unsubscribe();
  }

  void CascadedDeepBallTracking::detection_tracking_process(cv::Mat& src_img)
  {
    /* detect and track the drone */
    double t = ros::Time::now().toSec();
    SingleObjectDeepTrackingDetection::detection_tracking_process(src_img);

    //ROS_INFO("binding_model_detection: %f", ros::Time::now().toSec() - t);

    if(!detected_)
      {
        /* TODO: recovering by separate model detection */
        ball_depth_ = -1;
        return ;
      }

    double ball_depth = 0;
    bool ball_detection = boundingboxDetection(src_img, ball_depth);

    /* only use ball radius information */
    ballPositionFromDepth(ball_detection, ball_depth, false, 0);
    //ROS_WARN("                     total detection: %f", ros::Time::now().toSec() - t);
  }

  bool CascadedDeepBallTracking::boundingboxDetection(cv::Mat& src_img, double& ball_depth)
  {
    detected_cnt ++;

    /* do cascaded deep detection */
    double t = ros::Time::now().toSec();
    auto search_area = best_detection_candidate_.corners;
    cv::Mat temp_img;
    expandedBoundingImage(src_img, search_area, 1.1, temp_img, search_area); // expand, with hyper parameter
    cropped_img_ = temp_img.clone();
    /* TODO: check the relation between keep_aspect_ratio_in_inference and ball detection */
    bool keep_aspect_ratio_in_inference_temp = keep_aspect_ratio_in_inference_;
    keep_aspect_ratio_in_inference_ = true;
    auto detection_candidates = deepDetectionCore(seperated_model_detection_engine_, cropped_img_, separate_model_detection_score_threshold_, 10);
    keep_aspect_ratio_in_inference_ = keep_aspect_ratio_in_inference_temp;

    //ROS_INFO("seperated_model_detection: %f", ros::Time::now().toSec() - t);

    coral::DetectionCandidate ball_candidate; // 0
    coral::DetectionCandidate drone_candidate; // 1
    int ball_num = 0;
    int drone_num = 0;
    for (const auto& candidate:  detection_candidates)
      {
        /*
          ROS_INFO("id: %d, score: %f, [%f, %f, %f, %f]",
          candidate.label, candidate.score,
          candidate.corners.xmin, candidate.corners.ymin,
          candidate.corners.xmax, candidate.corners.ymax);
        */
        if(candidate.label == CLASS_ID::BALL)
          {
            ball_num++;
            if (ball_candidate.score < candidate.score) ball_candidate = candidate;
          }
        if(candidate.label == CLASS_ID::DRONE)
          {
            drone_num++;
            if (drone_candidate.score < candidate.score) drone_candidate = candidate;
          }
      }

    if(ball_num > 0)
      {
        /* calculate the ball radius */
        if(ball_candidate.corners.xmin > 0 && ball_candidate.corners.ymin > 0 &&
           ball_candidate.corners.xmax < cropped_img_.size().width && ball_candidate.corners.ymax < cropped_img_.size().height)
          {
            ball_pixel_radius_ = (ball_candidate.corners.xmax - ball_candidate.corners.xmin + ball_candidate.corners.ymax - ball_candidate.corners.ymin) / 4;
          }
        else if(ball_candidate.corners.xmin == 0 && ball_candidate.corners.ymin > 0 &&
                ball_candidate.corners.xmax < cropped_img_.size().width && ball_candidate.corners.ymax < cropped_img_.size().height)
          {
            ball_pixel_radius_ = (ball_candidate.corners.ymax - ball_candidate.corners.ymin) / 2;
          }
        else if(ball_candidate.corners.xmin > 0 && ball_candidate.corners.ymin > 0 &&
                ball_candidate.corners.xmax == cropped_img_.size().width && ball_candidate.corners.ymax < cropped_img_.size().height)
          {
            ball_pixel_radius_ = (ball_candidate.corners.ymax - ball_candidate.corners.ymin) / 2;
          }
        else if(ball_candidate.corners.xmin > 0 && ball_candidate.corners.ymin == 0 &&
                ball_candidate.corners.xmax < cropped_img_.size().width && ball_candidate.corners.ymax < cropped_img_.size().height)
          {
            //ROS_INFO("okokokoko");
            ball_pixel_radius_ = (ball_candidate.corners.xmax - ball_candidate.corners.xmin) / 2;
          }
        else if(ball_candidate.corners.xmin > 0 && ball_candidate.corners.ymin > 0 &&
                ball_candidate.corners.xmax < cropped_img_.size().width && ball_candidate.corners.ymax == cropped_img_.size().height)
          {
            ball_pixel_radius_ = (ball_candidate.corners.xmax - ball_candidate.corners.xmin) / 2;
          }
        else
          {
            ball_pixel_radius_ = -1;
            return false;
          }

        cv::rectangle(src_img,
                      cv::Point(ball_candidate.corners.xmin + search_area.xmin,
                                ball_candidate.corners.ymin + search_area.ymin),
                      cv::Point(ball_candidate.corners.xmax + search_area.xmin,
                                ball_candidate.corners.ymax + search_area.ymin),
                      cv::Scalar(255, 0, 0), 2);

        cv::rectangle(cropped_img_,
                      cv::Point(ball_candidate.corners.xmin, ball_candidate.corners.ymin),
                      cv::Point(ball_candidate.corners.xmax, ball_candidate.corners.ymax),
                      cv::Scalar(255, 0, 0), 2);

        ball_cnt ++;
      }
    else
      {
        //ROS_WARN("No ball is detected in the bounding box");
        ball_pixel_radius_ = -1;
        return false;
      }

    if(drone_num > 0)
      {
        cv::rectangle(src_img,
                      cv::Point(drone_candidate.corners.xmin + search_area.xmin,
                                drone_candidate.corners.ymin + search_area.ymin),
                      cv::Point(drone_candidate.corners.xmax + search_area.xmin,
                                drone_candidate.corners.ymax + search_area.ymin),
                      cv::Scalar(255, 0, 0), 2);

        cv::rectangle(cropped_img_,
                      cv::Point(drone_candidate.corners.xmin, drone_candidate.corners.ymin),
                      cv::Point(drone_candidate.corners.xmax, drone_candidate.corners.ymax),
                      cv::Scalar(255, 0, 0), 2);

        drone_cnt ++;
      }
    else
      {
        //ROS_WARN("No drone is detected in the bounding box");
      }

    /* 3d position of ball */
    if(f_dash_ == 0)
      {
        ball_pixel_radius_ = -1;
        return false;
      }

    ball_depth = f_dash_ * ball_real_radius_ / ball_pixel_radius_;
    return true;

    // ROS_INFO("ball cnt: %d [%f]; drone cnt: %d [%f], detected cnt: %d", ball_cnt, ball_cnt / (float)detected_cnt, drone_cnt, drone_cnt / (float)detected_cnt, detected_cnt);
  }

  void CascadedDeepBallTracking::publish(const std_msgs::Header& msg_header, const cv::Mat& src_img)
  {
    ColorFilterBallTracking::publish(msg_header, src_img);
    debug_image_pub_.publish(cv_bridge::CvImage(msg_header, sensor_msgs::image_encodings::RGB8, cropped_img_).toImageMsg());
  }

} //namespace edgetpu_roscpp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (edgetpu_roscpp::CascadedDeepBallTracking, nodelet::Nodelet);
