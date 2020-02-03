#include <mbzirc2020_task1_tasks/RansacLineFitting.h>

RansacLineFitting::RansacLineFitting(ros::NodeHandle nh, ros::NodeHandle nhp){
  nh_ = nh;
  nhp_ = nhp;

  std::string target_pt_sub_topic_name;
  nhp_.param("target_point_sub_topic_name", target_pt_sub_topic_name, std::string("/treasure/point_detected"));
  nhp_.param("cand_points2d_initial_size", cand_points2d_init_size_, 15);
  nhp_.param("cand_points3d_initial_size", cand_points3d_init_size_, 15);
  nhp_.param("cand_points2d_max_size", cand_points2d_max_size_, 50);
  nhp_.param("cand_points3d_max_size", cand_points3d_max_size_, 50);
  nhp_.param("ransac_visualization_flag", ransac_vis_flag_, true);
  nhp_.param("ransac_3d_mode", ransac_3d_mode_, false);
  nhp_.param("target_point_maximum_disappear_time", target_pt_dispear_time_thre_, 0.8);
  nhp_.param("target_close_distance_threshold", target_close_dist_thre_, 1.0);
  nhp_.param("lpf_z_gain", lpf_z_gain_, 0.8);
  nhp_.param("yaw_diff_threshold", yaw_diff_thre_, M_PI / 3.0);

  initializeEstimatorParam();
  estimator_state_ = STOP_ESTIMATION;

  estimator_2d_.Initialize(0.1, 100); // Threshold, iterations
  estimator_3d_.Initialize(0.1, 100); // Threshold, iterations

  target_pt_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(target_pt_sub_topic_name, 1, &RansacLineFitting::targetPointCallback, this, ros::TransportHints().tcpNoDelay());

  fitted_line_pub_ = nh_.advertise<visualization_msgs::Marker>("/treasure/line_fitting_marker", 1);
  fitted_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/treasure/points_fitting_markers", 1);
}

void RansacLineFitting::targetPointCallback(const geometry_msgs::PointStampedConstPtr & msg){
  if (estimator_state_ == STOP_ESTIMATION)
    return;
  double cur_time = msg->header.stamp.toSec();
  if (target_pt_update_time_ < 0.0) // when in initial case
    target_pt_update_time_ = cur_time;
  else if (fabs(cur_time - target_pt_update_time_) > target_pt_dispear_time_thre_)
    initializeEstimatorParam();
  else
    target_pt_update_time_ = cur_time;

  std::shared_ptr<GRANSAC::AbstractParameter> CandPt2d = std::make_shared<Point2D>(msg->point.x, msg->point.y);
  cand_points2d_.push_back(CandPt2d);
  std::shared_ptr<GRANSAC::AbstractParameter> CandPt3d = std::make_shared<Point3D>(msg->point.x, msg->point.y, msg->point.z);
  cand_points3d_.push_back(CandPt3d);
  if (lpf_z_ < 0.0) // not initialize
    lpf_z_ = msg->point.z;
  else
    lpf_z_ = lpf_z_ * lpf_z_gain_ + (1 - lpf_z_gain_) * msg->point.z; // low-pass filter

  if (cand_points3d_.size() > cand_points3d_init_size_){
    while (cand_points3d_.size() > cand_points3d_max_size_)
      cand_points3d_.erase(cand_points3d_.begin());
    // todo: update trigger by time event or subscribe
    if (ransac_3d_mode_)
      update();
  }
  if (cand_points2d_.size() > cand_points2d_init_size_){
    while (cand_points2d_.size() > cand_points2d_max_size_)
      cand_points2d_.erase(cand_points2d_.begin());
    // todo: update trigger by time event or subscribe
    if (!ransac_3d_mode_)
      update();
  }
}

void RansacLineFitting::initializeEstimatorParam(){
    target_pt_update_time_ = -1.0;
    // initalize ransac related variables to re-start estimation
    lpf_z_ = -1;
    cand_points3d_.clear();
    cand_points2d_.clear();
    estimator_state_ = PAUSE_ESTIMATION;
    ROS_INFO("[RansacLineFitting] Ransac estimation initialized");
}

bool RansacLineFitting::isEstimated(){
  if (estimator_state_ == PAUSE_ESTIMATION || estimator_state_ == STOP_ESTIMATION)
    return false;
  else if (estimator_state_ == IN_ESTIMATION){
    double cur_time = ros::Time::now().toSec();
    if (fabs(cur_time - target_pt_update_time_) > target_pt_dispear_time_thre_){
      initializeEstimatorParam();
      return false;
    }
    else
      return true;
  }
}

void RansacLineFitting::update(){
  if (estimator_state_ == PAUSE_ESTIMATION){
    estimator_state_ = IN_ESTIMATION;
    ROS_INFO("[RansacLineFitting] Ransac estimation starts");
  }
  // ROS_INFO("Estimator.Initialize");
  if (ransac_3d_mode_)
    estimator_3d_.Estimate(cand_points3d_);
  else
    estimator_2d_.Estimate(cand_points2d_);
  // ROS_INFO("Estimator.Estimate");

  visualizeRansacLine();
  if (ransac_vis_flag_)
    visualizeRansacInliers();
}

bool RansacLineFitting::checkEstimationWithYawAng(double yaw){
  if (ransac_3d_mode_){
    // to develop
    return false;
  }
  else{
    double slope = estimator_2d_.GetBestModel()->getSlope();
    double slope_diff = fabs(yaw - M_PI - slope);
    while (slope_diff > M_PI)
      slope_diff -= 2 * M_PI;
    if (fabs(slope_diff) > yaw_diff_thre_){
      // debug
      // std::cout << "slope: " << slope << ", yaw: " << yaw << "\n";
      return false;
    }
    else
      return true;
  }
}

bool RansacLineFitting::getNearestWaypoint(Eigen::Vector3d pos, Eigen::Vector3d &waypt){
  if (ransac_3d_mode_){
    // to develop
    return false;
  }
  else{
    std::shared_ptr<GRANSAC::AbstractParameter> pos2d = std::make_shared<Point2D>(pos[0], pos[1]);
    auto waypt2d = std::dynamic_pointer_cast<Point2D>(estimator_2d_.GetBestModel()->ComputeNearestPoint(pos2d));
    waypt = Eigen::Vector3d(waypt2d->m_Point2D[0], waypt2d->m_Point2D[1], lpf_z_);
    return true;
  }
}

bool RansacLineFitting::isNearTarget(Eigen::Vector3d pos){
  int checked_point_cnt = 5;
  if (ransac_3d_mode_){
    // to develop
    return false;
  }
  else{
    if (checked_point_cnt > cand_points2d_.size()) checked_point_cnt = cand_points2d_.size();
    int near_point_cnt = 0;
    for (int i = 0; i < checked_point_cnt; ++i){
      auto pt = std::dynamic_pointer_cast<Point2D>(cand_points2d_[cand_points2d_.size() - 1 - i]);
      double dist = sqrt(pow(pos[0] - pt->m_Point2D[0], 2.0) + pow(pos[1] - pt->m_Point2D[1], 2.0));
      if (dist < 0.7 + target_close_dist_thre_)
        ++near_point_cnt;
    }
    if (near_point_cnt >= checked_point_cnt / 2 + 1)
      return true;
    else
      return false;
  }
}

void RansacLineFitting::stopEstimation(){
  initializeEstimatorParam();
  estimator_state_ = STOP_ESTIMATION;
}

void RansacLineFitting::startEstimation(){
  initializeEstimatorParam();
}

void RansacLineFitting::visualizeRansacLine(){
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "treasure_marker";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Line strip is blue
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  // Create the vertices for the points and lines
  double predict_factor = 1.5; // predict_horizon = history_horizon x predict_factor
  if (ransac_3d_mode_){
    auto ransac_start_pt = std::dynamic_pointer_cast<Point3D>(estimator_3d_.GetBestModel()->GetModelParams()[0]);
    auto ransac_end_pt = std::dynamic_pointer_cast<Point3D>(estimator_3d_.GetBestModel()->GetModelParams()[1]);
    Point3D slope = (*ransac_end_pt) - (*ransac_start_pt);
    int rate_id = 0; // chosen the slope with large step to reduce errors
    for (int i = 1; i < 3; ++i){
      if (fabs(slope.m_Point3D[i]) > fabs(slope.m_Point3D[rate_id]))
        rate_id = i;
    }
    Point3D line_start_pt = (*ransac_start_pt) +
      slope * ((std::dynamic_pointer_cast<Point3D>(cand_points3d_[rate_id])->m_Point3D[rate_id] - ransac_start_pt->m_Point3D[rate_id]) / slope.m_Point3D[rate_id]);
    Point3D line_end_pt = (*ransac_end_pt) +
      slope * ((std::dynamic_pointer_cast<Point3D>(cand_points3d_[cand_points3d_.size() - 1])->m_Point3D[rate_id] - ransac_end_pt->m_Point3D[rate_id]
                + predict_factor * (std::dynamic_pointer_cast<Point3D>(cand_points3d_[cand_points3d_.size() - 1])->m_Point3D[rate_id] - std::dynamic_pointer_cast<Point3D>(cand_points3d_[rate_id])->m_Point3D[rate_id])
                ) / slope.m_Point3D[rate_id]);
    geometry_msgs::Point p;
    p.x = line_start_pt.m_Point3D[0];
    p.y = line_start_pt.m_Point3D[1];
    p.z = line_start_pt.m_Point3D[2];
    line_strip.points.push_back(p);
    p.x = line_end_pt.m_Point3D[0];
    p.y = line_end_pt.m_Point3D[1];
    p.z = line_end_pt.m_Point3D[2];
    line_strip.points.push_back(p);
  }
  else{
    auto ransac_start_pt = std::dynamic_pointer_cast<Point2D>(estimator_2d_.GetBestModel()->GetModelParams()[0]);
    auto ransac_end_pt = std::dynamic_pointer_cast<Point2D>(estimator_2d_.GetBestModel()->GetModelParams()[1]);
    Point2D slope = (*ransac_end_pt) - (*ransac_start_pt);
    int rate_id = 0; // chosen the slope with large step to reduce errors
    if (fabs(slope.m_Point2D[1]) > fabs(slope.m_Point2D[rate_id]))
      rate_id = 1;
    Point2D line_start_pt = (*ransac_start_pt) +
      slope * ((std::dynamic_pointer_cast<Point2D>(cand_points2d_[rate_id])->m_Point2D[rate_id] - ransac_start_pt->m_Point2D[rate_id]) / slope.m_Point2D[rate_id]);
    Point2D line_end_pt = (*ransac_end_pt) +
      slope * ((std::dynamic_pointer_cast<Point2D>(cand_points2d_[cand_points2d_.size() - 1])->m_Point2D[rate_id] - ransac_end_pt->m_Point2D[rate_id]
                + predict_factor * (std::dynamic_pointer_cast<Point2D>(cand_points2d_[cand_points2d_.size() - 1])->m_Point2D[rate_id] - std::dynamic_pointer_cast<Point2D>(cand_points2d_[rate_id])->m_Point2D[rate_id])
                ) / slope.m_Point2D[rate_id]);
    geometry_msgs::Point p;
    p.x = line_start_pt.m_Point2D[0];
    p.y = line_start_pt.m_Point2D[1];
    p.z = lpf_z_;
    line_strip.points.push_back(p);
    p.x = line_end_pt.m_Point2D[0];
    p.y = line_end_pt.m_Point2D[1];
    p.z = lpf_z_;
    line_strip.points.push_back(p);
  }
  fitted_line_pub_.publish(line_strip);
}

void RansacLineFitting::visualizeRansacInliers(){
  visualization_msgs::MarkerArray ransac_points;
  visualization_msgs::Marker pt;
  pt.header.frame_id = "world";
  pt.header.stamp = ros::Time::now();
  pt.ns = "treasure_marker";
  pt.action = visualization_msgs::Marker::ADD;
  pt.pose.orientation.w = 1.0;
  pt.id = 0;
  pt.type = visualization_msgs::Marker::SPHERE;

  // PT/LINE_LIST markers use only the x component of scale, for the line width
  pt.scale.x = 0.1;
  pt.scale.y = 0.1;
  pt.scale.z = 0.1;

  // Line strip is blue
  pt.color.r = 1.0;
  pt.color.a = 0.4;

  if (ransac_3d_mode_){
    auto start_cand_pt = std::dynamic_pointer_cast<Point3D>(cand_points3d_[0]);
    auto end_cand_pt = std::dynamic_pointer_cast<Point3D>(cand_points3d_[cand_points3d_.size() - 1]);
    for (int i = 0; i < cand_points3d_.size(); ++i){
      auto pt_3d = std::dynamic_pointer_cast<Point3D>(cand_points3d_[i]);
      pt.pose.position.x = pt_3d->m_Point3D[0];
      pt.pose.position.y = pt_3d->m_Point3D[1];
      pt.pose.position.z = pt_3d->m_Point3D[2];
      ransac_points.markers.push_back(pt);
      pt.id += 1;
    }
  }
  else{
    auto start_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points2d_[0]);
    auto end_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points2d_[cand_points2d_.size() - 1]);
    for (int i = 0; i < cand_points2d_.size(); ++i){
      auto pt_2d = std::dynamic_pointer_cast<Point2D>(cand_points2d_[i]);
      pt.pose.position.x = pt_2d->m_Point2D[0];
      pt.pose.position.y = pt_2d->m_Point2D[1];
      pt.pose.position.z = lpf_z_; // todo
      ransac_points.markers.push_back(pt);
      pt.id += 1;
    }
  }

  pt.color.r = 0.0;
  pt.color.b = 1.0;
  pt.color.a = 1.0;
  if (ransac_3d_mode_){
    auto BestInliers = estimator_3d_.GetBestInliers();
    if (BestInliers.size() > 0)
      {
        for (auto& Inlier : BestInliers)
          {
            auto RPt = std::dynamic_pointer_cast<Point3D>(Inlier);
            pt.pose.position.x = RPt->m_Point3D[0];
            pt.pose.position.y = RPt->m_Point3D[1];
            pt.pose.position.z = RPt->m_Point3D[2];
            ransac_points.markers.push_back(pt);
            pt.id += 1;
          }
      }
  }
  else{
    auto BestInliers = estimator_2d_.GetBestInliers();
    if (BestInliers.size() > 0)
      {
        for (auto& Inlier : BestInliers)
          {
            auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
            pt.pose.position.x = RPt->m_Point2D[0];
            pt.pose.position.y = RPt->m_Point2D[1];
            pt.pose.position.z = lpf_z_; // todo
            ransac_points.markers.push_back(pt);
            pt.id += 1;
          }
      }
  }
  fitted_points_pub_.publish(ransac_points);
}
