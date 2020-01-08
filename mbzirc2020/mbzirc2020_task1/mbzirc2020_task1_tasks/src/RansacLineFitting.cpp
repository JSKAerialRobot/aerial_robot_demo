#include <mbzirc2020_task1_tasks/RansacLineFitting.h>

RansacLineFitting::RansacLineFitting(ros::NodeHandle nh, ros::NodeHandle nhp){
  nh_ = nh;
  nhp_ = nhp;

  std::string target_pt_sub_topic_name;
  nhp_.param("target_point_sub_topic_name", target_pt_sub_topic_name, std::string("/treasure/point_detected"));
  nhp_.param("cand_points_max_size", cand_points_max_size_, 50);
  nhp_.param("ransac_visualization_flag", ransac_vis_flag_, true);

  lpf_z_ = -1;
  lpf_z_gain_ = 0.8;

  estimator_2d_.Initialize(1.4, 100); // Threshold, iterations

  target_pt_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(target_pt_sub_topic_name, 1, &RansacLineFitting::targetPointCallback, this, ros::TransportHints().tcpNoDelay());

  fitted_line_pub_ = nh_.advertise<visualization_msgs::Marker>("/treasure/line_fitting_marker", 1);
  fitted_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/treasure/points_fitting_markers", 1);
}

void RansacLineFitting::targetPointCallback(const geometry_msgs::PointStampedConstPtr & msg){
  std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(msg->point.x, msg->point.y);
  cand_points_.push_back(CandPt);
  if (lpf_z_ < 0.0) // not initialize
    lpf_z_ = msg->point.z;
  else
    lpf_z_ = lpf_z_ * lpf_z_gain_ + (1 - lpf_z_gain_) * msg->point.z; // low-pass filter
  if (cand_points_.size() > cand_points_max_size_){
    cand_points_.erase(cand_points_.begin());
    // todo: update trigger by time event or subscribe
    update();
  }
}

void RansacLineFitting::update(){
  ROS_INFO("Estimator.Initialize");
  estimator_2d_.Estimate(cand_points_);
  ROS_INFO("Estimator.Estimate");

  visualizeRansacLine();
  if (ransac_vis_flag_)
    visualizeRansacInliers();
}

void RansacLineFitting::visualizeRansacLine(){
  auto BestLine = estimator_2d_.GetBestModel();

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
  std::vector<geometry_msgs::Point> ransac_end_points;
  for (uint32_t i = 0; i < 2; ++i)
    {
      auto BestLinePt = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[i]);
      geometry_msgs::Point p;
      p.x = BestLinePt->m_Point2D[0];
      p.y = BestLinePt->m_Point2D[1];
      p.z = lpf_z_;
      ransac_end_points.push_back(p);
    }
  double slope_rate = (ransac_end_points[1].y - ransac_end_points[0].y) / (ransac_end_points[1].x - ransac_end_points[0].x);
  auto start_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points_[0]);
  auto end_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points_[cand_points_max_size_ - 1]);
  for (int i = 0; i < 2; ++i){
    geometry_msgs::Point p;
    if (i == 0)
      p.x = start_cand_pt->m_Point2D[0];
    else
      p.x = (end_cand_pt->m_Point2D[0] - start_cand_pt->m_Point2D[0]) * 2.0 + end_cand_pt->m_Point2D[0];
    p.y = ransac_end_points[i].y + slope_rate * (p.x - ransac_end_points[i].x);
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

  auto start_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points_[0]);
  auto end_cand_pt = std::dynamic_pointer_cast<Point2D>(cand_points_[cand_points_max_size_ - 1]);
  for (int i = 0; i < cand_points_max_size_; ++i){
    auto pt_2d = std::dynamic_pointer_cast<Point2D>(cand_points_[i]);
    pt.pose.position.x = pt_2d->m_Point2D[0];
    pt.pose.position.y = pt_2d->m_Point2D[1];
    pt.pose.position.z = lpf_z_; // todo
    ransac_points.markers.push_back(pt);
    pt.id += 1;
  }

  pt.color.r = 0.0;
  pt.color.b = 1.0;
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
  fitted_points_pub_.publish(ransac_points);
}
