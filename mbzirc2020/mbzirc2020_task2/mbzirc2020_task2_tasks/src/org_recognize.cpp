#include <mbzirc2020_task2_tasks/org_recognize.h>
#include <mbzirc2020_task2_tasks/kmeans.h>
#include <mbzirc2020_task2_tasks/space_detector.h>

namespace mbzirc2020_task2_tasks
{

  void RedObjectDetectionWithHSVFilter::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("init_x", init_x, 0.0);
    pnh_->param("init_y", init_y, 0.0);
    pnh_->param("init_z", init_z, 0.0);
    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id_", frame_id_, std::string("target_object"));
    always_subscribe_ = true;

    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/uav/nav"), 1);
    object_marker_pub = advertise<visualization_msgs::Marker>(*nh_, frame_id_ + std::string("/visualize"), 1);
    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    xy_pub = advertise<geometry_msgs::Point>(*nh_, "xy_point", 1);
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);
    area_pub = advertise<std_msgs::Float64>(*nh_, "area", 1);
    marker_pub = advertise<visualization_msgs::Marker>(*nh_, "marker", 1);
    angle_pub = advertise<std_msgs::Float64>(*nh_, "target_object/angle", 1);
    working_phase_pub = advertise<std_msgs::Int32>(*nh_, "working_phase/cpp", 1);
    target_pos_pub = advertise<aerial_robot_msgs::FlightNav>(*nh_, "/uav/nav", 1);
    jointstate_pub = advertise<sensor_msgs::JointState>(*nh_, "/hydrusx/joints_ctrl", 1);

    working_phase = 0;
    working_phase_py = 0;

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void RedObjectDetectionWithHSVFilter::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &RedObjectDetectionWithHSVFilter::imageCallback, this);
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &RedObjectDetectionWithHSVFilter::cameraInfoCallback, this);
    plane_sub_ = nh_->subscribe("polygon_array", 1, &RedObjectDetectionWithHSVFilter::planeCallback, this);
    image_depth_sub_ = it_->subscribe("image_depth", 1, &RedObjectDetectionWithHSVFilter::imagedepthCallback, this);
    working_phase_sub_ = nh_->subscribe("working_phase/py", 1, &RedObjectDetectionWithHSVFilter::workingphaseCallback, this);
    uav_odom_sub_ =  nh_->subscribe("/uav/baselink/odom", 1, &RedObjectDetectionWithHSVFilter::uavodomCallback, this);

  }

  void RedObjectDetectionWithHSVFilter::openjoints()
  {
    sensor_msgs::JointState msg;
    // // msg.position = [1.40, 1.57, 1.40];
    // msg.position.push_back(1.40);
    // msg.position.push_back(1.40);
    
    // jointstate_pub.publish(msg);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // // msg.position = [1.25, 1.57, 1.25];
    // msg.position.push_back(1.25);
    // msg.position.push_back(1.25);
    
    // jointstate_pub.publish(msg);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // msg.position = [1.0, 1.57, 1.0];
    msg.position.push_back(1.0);
    msg.position.push_back(1.0);    
    jointstate_pub.publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
  }

  //   void RedObjectDetectionWithHSVFilter::closejoints()
  // {
  //   sensor_msgs::JointState msg;
  // }

  void RedObjectDetectionWithHSVFilter::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void RedObjectDetectionWithHSVFilter::uavodomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    hydrus_angle = yaw;

    hydrus_x = msg->pose.pose.position.x;
    hydrus_y = msg->pose.pose.position.y;
    hydrus_z = msg->pose.pose.position.z;
  }

  void RedObjectDetectionWithHSVFilter::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K_normal(msg->K[0], msg->K[1], msg->K[2],
                            msg->K[3], msg->K[4], msg->K[5],
                            msg->K[6], msg->K[7], msg->K[8]);
    camera_K_inv_ = camera_K_normal.inverse();
    camera_K = camera_K_normal;
    real_size_scale_ = msg->K[0] * msg->K[4];
    object_distance = 18.0;  // change according to the target distance
    camdep.setCameraInfo(*msg);
    cam_info_sub_.shutdown();
  }

  void RedObjectDetectionWithHSVFilter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(real_size_scale_ == 0) return; // no receive camera_info yet.

    tf2::Transform cam_tf;
    try{
      //geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", msg->header.stamp);
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", ros::Time(0));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    
    tf2::Transform leg_cog_tf;
    try{
      geometry_msgs::TransformStamped leg_cog_msg = tf_buff_.lookupTransform("cog", "leg5", ros::Time(0));

      tf2::convert(leg_cog_msg.transform, leg_cog_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf2::Vector3 leg_cog_translation = leg_cog_tf.getOrigin();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat src_image = cv_ptr->image;

    // detect the target objects

    if(working_phase == 0 && working_phase_py == -1){
      cv::Mat hsv_image;
      cvtColor(src_image, hsv_image, cv::COLOR_BGR2HSV);

      cv::Mat mask = cv::Mat::zeros(src_image.rows, src_image.cols, CV_8UC3);

      DataFrame data;
      int i_count = 0;  //  highest point
      geometry_msgs::Point xy_center;

      // to find red objects using BGR -> HSV

      for (int i = 0; i < src_image.rows; i++){
        cv::Vec3b * ms = mask.ptr<cv::Vec3b>(i);
        cv::Vec3b * chan = hsv_image.ptr<cv::Vec3b>(i);
        for (int j = 0; j < src_image.cols; j++){
          cv::Vec3b bgr = ms[j];
          cv::Vec3b hsv = chan[j];
          if((hsv[0] < 20) || (hsv[0] > 150)){
            if ((hsv[1] > 128) && (hsv[2] > 128)){
              ms[j] = cv::Vec3b(255, 255, 255);
              Pp p = {(double)j, (double)i};
              data.push_back(p);
              if(i_count == 0){
                xy_center.x = j;
                xy_center.y = i;
                xy_center.z = 1;
                i_count ++;
              }
            }
          }
        }
      }

      // // contour approximation
      // cv::Mat graymask;
      // cv::cvtColor(mask, graymask, cv::COLOR_BGR2GRAY);
      // std::vector<std::vector<cv::Point> > contours;
      // std::vector<cv::Vec4i> hierarchy;
      // cv::findContours(graymask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

      // double max_contour_area = 0;
      // std::vector<cv::Point> target_contour;

      // auto calc_position = [](std::vector<cv::Point> contour) {
      //                        cv::Moments contour_moments = cv::moments(contour, true);
      //                        tf2::Vector3 pos;
      //                        pos.setX(contour_moments.m10 / contour_moments.m00);
      //                        pos.setY(contour_moments.m01 / contour_moments.m00);
      //                        return pos;
      //                      };

      // for(const auto& contour : contours) {
      //   double real_contour_area = cv::contourArea(contour);
      //   NODELET_DEBUG_STREAM("contour size" << real_contour_area);

      //   if(true) {
      //     /* find maximum size */
      //     if (max_contour_area < real_contour_area) {
      //       max_contour_area = real_contour_area;
      //       target_contour = contour;
      //     }
      //   }
      // }

      // tf2::Vector3 target_obj_uv = calc_position(target_contour);
      tf2::Vector3 target_obj_uv;
      if (true) {
        if (debug_view_) {
          cv::Mat debug_image = cv::Mat::zeros(src_image.rows, src_image.cols, CV_8UC3);
        }
      }

      if(data.size() > 10) {
        Pp means = k_means(data, 1, 100);
        xy_center.x = means.x;
        xy_center.y = means.y;
        xy_center.z = 1.0;

        cv::circle(cv_ptr->image, cv::Point(xy_center.x, xy_center.y), 3, cv::Scalar(0,255,0), 6, 3);
      }
      else{
        xy_center.x = 0.0;
        xy_center.y = 0.0;
        xy_center.z = 0.0;
      }
      xy_pub.publish(xy_center);

      target_obj_uv.setX(xy_center.x);
      target_obj_uv.setY(xy_center.y);
      target_obj_uv.setZ(1.0);

      tf2::Vector3 object_pos_in_optical_frame = camera_K_inv_ * target_obj_uv * object_distance;

      geometry_msgs::TransformStamped obj_tf_msg;
      obj_tf_msg.header = msg->header;
      obj_tf_msg.header.frame_id = std::string("world");
      obj_tf_msg.child_frame_id = frame_id_;
      obj_tf_msg.transform.translation = tf2::toMsg(cam_tf * object_pos_in_optical_frame);
      obj_tf_msg.transform.rotation.w = 1.0;
      tf_br_.sendTransform(obj_tf_msg);

      geometry_msgs::Vector3Stamped obj_pos_msg;
      obj_pos_msg.header = obj_tf_msg.header;
      obj_pos_msg.vector = obj_tf_msg.transform.translation;

      // go to approach phase

      working_phase_msg.data = 0;
      working_phase_pub.publish(working_phase_msg);

    }

    // draw plane points(maybe not necessarry)
    if(planes_img.size() != 0){
      for(const auto plane_img : planes_img){
        int x = 0;
        int y = 0;
        for(int i = 0; i != plane_img.size(); i++){
          if(i % 2 == 0){
            x = plane_img[i];
          }
          else{
            y = plane_img[i];
          }
        }
      }
      planes_img.erase(planes_img.begin(), planes_img.end());
    }

    // draw polygon line
    if(working_phase == 3){
      int count = 0;
      if(vertice_highest.size() != 0){
        for(const auto v_highest : vertice_highest){
        jsk_recognition_utils::Polygon line_img_polygon(v_highest);
        cv::Mat canvas(src_image.rows, src_image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
        line_img_polygon.drawLineToImage(camdep, cv_ptr->image, cv::Scalar(0, 255, 0), 1);
        line_img_polygon.drawLineToImage(camdep, canvas, cv::Scalar(0, 255, 0), 1);
        if(count == 0){
          cv::imwrite("/home/kuromiya/draw_plane.png", canvas);
        }

        if(depth_img.cols != 0 && depth_img.rows != 0){
          // std::vector<float> answer = space_detector(canvas, depth_img);
	  std::vector<float> answer = space_detector(line_img_polygon, depth_img, hydrus_z, camdep);
	  double target_distance_z = v_highest[0].z();  // distane to target object
	  // std::vector<float> answer = space_detector_ground(canvas, depth_img, target_distance_z);
	  int graspable = int(answer[0]);
          if(graspable == 0){  // answer[0]
            continue;
          }
          else if(graspable == 1){  // answer[0]
	    ROS_INFO("good! : %d", int(answer[0]));
	
            float x = line_img_polygon.centroid()(0);
            float y = line_img_polygon.centroid()(1);
            float z = line_img_polygon.centroid()(2);

            tf2::Vector3 target_xyz;  // target position on cam_tf
            target_xyz.setX(x);
            target_xyz.setY(y);
            target_xyz.setZ(z);

	    cam_target_xyz = target_xyz;
	    
            geometry_msgs::Vector3Stamped tar_pos_msg1;
            tar_pos_msg1.header = msg->header;
            tar_pos_msg1.vector = tf2::toMsg(cam_tf * target_xyz);
            target_pos_pub_.publish(tar_pos_msg1);

            ROS_INFO("object pos w.r.t cam frame: [%f, %f, %f]",
                     target_xyz.x(), target_xyz.y(), target_xyz.z());

            ROS_INFO("object center w.r.t world: [%f, %f, %f]",
                     tar_pos_msg1.vector.x,
                     tar_pos_msg1.vector.y,
                     tar_pos_msg1.vector.z);


	    visualization_msgs::Marker marker_obj_msg;
	    marker_obj_msg.header.frame_id = "/world";
	    marker_obj_msg.header.stamp = msg->header.stamp;
	    marker_obj_msg.ns = "basic_shapes";

	    marker_obj_msg.type = visualization_msgs::Marker::CUBE;
	    marker_obj_msg.action = visualization_msgs::Marker::ADD;
	    // marker_obj_msg.scale.x = 0.25;
	    // marker_obj_msg.scale.y = 0.25;
	    // marker_obj_msg.scale.z = 0.35;
            marker_obj_msg.scale.x = 0.20;
	    marker_obj_msg.scale.y = 0.20;
	    marker_obj_msg.scale.z = 0.30;
	    marker_obj_msg.pose.position.x = tar_pos_msg1.vector.x;
            marker_obj_msg.pose.position.y = tar_pos_msg1.vector.y;
            marker_obj_msg.pose.position.z = tar_pos_msg1.vector.z -0.15;
	    // marker_obj_msg.pose.orientation =  tf::createQuaternionMsgFromYaw(answer[0] * M_PI / 180 + hydrus_angle);
	    marker_obj_msg.pose.orientation =  tf::createQuaternionMsgFromYaw(answer[1] * M_PI / 180 + cam_angle);
            ROS_INFO("object yaw w.r.t robot: %f; robot yaw: %f", answer[1] * M_PI / 180, hydrus_angle);
	    marker_obj_msg.color.g = 1.0f;
	    marker_obj_msg.color.a = 1.0f;
	    object_marker_pub.publish(marker_obj_msg);
	
	    double angle_rad = answer[1] * M_PI / 180;
	
	    std_msgs::Float64 angle;
            angle.data = angle_rad;  // degree -> rad
	    // double target_angle = hydrus_angle + angle_rad + 1.57;
	    double target_angle = cam_angle + angle_rad;
	    angle.data = target_angle;  // degree -> rad
            angle_pub.publish(angle);
            answer.erase(answer.begin(), answer.end());
	    double margin = 0.7;  // distance from the target object to the destination
	    double target_x = x;
	    double target_y = y;
	    double target_z = z;

	    geometry_msgs::Vector3Stamped tar_pos_msg2;
            tar_pos_msg2.header = msg->header;
            tar_pos_msg2.vector = tf2::toMsg(cam_tf * target_xyz);
	
	    target_x = tar_pos_msg2.vector.x - margin * std::cos(target_angle);
	    target_y = tar_pos_msg2.vector.y - margin * std::sin(target_angle);
	    target_z = tar_pos_msg2.vector.z;

	    //  for motion
	    target_x_ = target_x;
	    target_y_ = target_y;
	    target_z_ = target_z;
	    target_angle_ = target_angle;
	
	    target_xyz.setX(target_x);
            target_xyz.setY(target_y);
            target_xyz.setZ(target_z);
            tar_pos_msg2.vector = tf2::toMsg(cam_tf * target_xyz);
	    target_pos_pub_.publish(tar_pos_msg2);
	
	    std::cout << "target_x : " << target_x_ << std::endl;
	    std::cout << "target_y : " << target_y_ << std::endl;
	    std::cout << "target_z : " << target_z_ << std::endl;
	    std::cout << "target_angle : " << target_angle_ - 0.78 << std::endl;

	    visualization_msgs::Marker marker;
	    marker.header.frame_id = "/world";
	    marker.header.stamp = ros::Time(0);
	    marker.ns = "basic_shapes";
	    marker.id = 0;

	    marker.type = visualization_msgs::Marker::ARROW;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.lifetime = ros::Duration();

	    marker.scale.x = 0.5;
	    marker.scale.y = 0.05;
	    marker.scale.z = 0.05;
	    marker.pose.position.x = target_x;
	    marker.pose.position.y = target_y;
	    marker.pose.position.z = target_z;
	    marker.pose.orientation = tf::createQuaternionMsgFromYaw(target_angle);
	    marker.color.b = 1.0f;
	    marker.color.a = 1.0f;
	    marker_pub.publish(marker);

          }
          image_pub_.publish(cv_ptr->toImageMsg());
          count++;
        }
	if(count == 1){
	  break;  //  graspable object once detected -> finish
	}
        }
      }
       vertice_highest.erase(vertice_highest.begin(), vertice_highest.end());

      working_phase_msg.data = 3;
      working_phase_pub.publish(working_phase_msg);
      working_phase = 4;
    }

    
    //  motion
    if(working_phase == 4){
      //  xy > rotate > z > go up
      
      target_angle_ -= 0.785 * 3;  // initially based on link2 -> link3

      int go_check = 0; // decide whether to go to the target
      int go_pos_limit = 30;
      std::vector<double> target_pos_v;
      target_pos_v.push_back(target_x_);
      target_pos_v.push_back(target_y_);
      target_pos_v.push_back(target_z_);
      
      if(target_pos_cnd.size() > 0){
	int cnt = 0;
	int add_check = 0;
	for(const auto trg : target_pos_cnd){
	  if(std::abs(trg[0] - target_x_) < 0.1 && std::abs(trg[1] - target_y_) < 0.1 && std::abs(trg[2] - target_z_) < 0.1){
	    target_count[cnt]++;
	    std::cout << "target_count: "<< target_count[cnt] << std::endl;
	    if(target_count[cnt] > go_pos_limit){  //  go to the target
	      go_check = 1;
	    }
	    add_check++;
	    break;
	  }
	  cnt++;
	}

	if(add_check == 0){
	  target_pos_cnd.push_back(target_pos_v);
	  target_angle_cnd.push_back(target_angle_);
	  target_count.push_back(1);
	}
      }
      
      else if(target_pos_cnd.size() == 0 && target_pos_v[0] != 0.0){
	target_pos_cnd.push_back(target_pos_v);
	target_angle_cnd.push_back(target_angle_);
	target_count.push_back(1);
      }

      if(go_check == 1){

	openjoints();
	
	aerial_robot_msgs::FlightNav msg;
      
	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;
	msg.target_pos_x = target_x_;
	msg.target_pos_y = target_y_;
	msg.target_pos_z = 2.5;
	msg.target_psi = target_angle_;

	std::cout << "target_angle : " << target_angle_ << std::endl;
	std::cout << "final x : " << msg.target_pos_x << std::endl;
	std::cout << "final y : " << msg.target_pos_y << std::endl;
	
	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;
	msg.pos_xy_nav_mode = msg.NO_NAVIGATION;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.NO_NAVIGATION;
	msg.target_psi = target_angle_;

	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;
	msg.pos_xy_nav_mode = msg.NO_NAVIGATION;
	msg.psi_nav_mode = msg.NO_NAVIGATION;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.target_pos_z = target_z_ + 1.0;

	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;	
	msg.target_pos_z = target_z_ + 0.5;

	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;	
	msg.target_pos_z = target_z_;

	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	msg.pos_xy_nav_mode = msg.POS_MODE;
	msg.psi_nav_mode = msg.POS_MODE;
	msg.pos_z_nav_mode = msg.POS_MODE;
	msg.header.stamp = ros::Time(0);
	msg.control_frame = 0;
	msg.target = 0;	
	msg.target_pos_z = target_z_ - 0.1;

	target_pos_pub.publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(10));

	// msg.pos_xy_nav_mode = msg.POS_MODE;
	// msg.psi_nav_mode = msg.POS_MODE;
	// msg.pos_z_nav_mode = msg.POS_MODE;
	// msg.header.stamp = ros::Time(0);
	// msg.control_frame = 0;
	// msg.target = 0;	
	// msg.target_pos_z = target_z_ + 1.0;

	// target_pos_pub.publish(msg);
	// std::this_thread::sleep_for(std::chrono::seconds(10));

      // closejoints();
      }
      
      working_phase = 0;
      
      if(go_check == 1){
	std::cout << "reach michael" << std::endl;
	working_phase = 0;
      }

    }

    //  xy and z adjust(p control)
    if(working_phase == 6){
      double target_x = cam_target_xyz.x();
      double target_y = cam_target_xyz.y();
      double target_z = std::abs(cam_target_xyz.z());

      double coef_x  = 0.1;
      double coef_y  = 0.2;
      double coef_z  = 0.3;

      double margin = 0.70;
      double limit = 0.10;

      // double dif_x = std::abs(target_x);
      // double dif_y = std::abs(target_y);
      // double dif_z = std::abs(target_z);

      tf2::Vector3 new_target_xyz;  // target position on cam_tf

      std::cout << "cam_target_x : "<< target_x << std::endl;
      std::cout << "cam_target_y : "<< target_y << std::endl;
      std::cout << "cam_target_z : "<< target_z << std::endl;

      tf2::Transform camera_base_tf;
      try{
	geometry_msgs::TransformStamped camera_base_msg = tf_buff_.lookupTransform("rs_d435_color_optical_frame", "link3", ros::Time(0));

	tf2::convert(camera_base_msg.transform, camera_base_tf);
      }
      catch (tf2::TransformException &ex) {
	ROS_WARN("%s", ex.what());
	return;
      }

      tf2::Vector3 camera_base_translation = camera_base_tf.getOrigin();
      geometry_msgs::Vector3Stamped camera_base_translation_world_msg;
      camera_base_translation_world_msg.header = msg->header;
      camera_base_translation_world_msg.vector = tf2::toMsg(cam_tf * camera_base_translation);
      
      if(std::abs(target_x) > limit){
	new_target_xyz.setX(coef_x * target_x);
	new_target_xyz.setY(0.0000);
	new_target_xyz.setZ(0.0000);

	std::cout << "x shift" << std::endl;
      }

      else if(std::abs(target_z - margin) > limit){
      	new_target_xyz.setX(0.00000);
	new_target_xyz.setY(0.00000);
      	new_target_xyz.setZ(coef_z * (target_z - margin));

	
      	std::cout << "y shift" << std::endl;
      }

      else if(std::abs(target_y) > limit){
      	new_target_xyz.setX(0.0);
      	new_target_xyz.setY(coef_y * target_y);
	new_target_xyz.setZ(0.0);

      	std::cout << "z shift" << std::endl;
      }

      else{
	new_target_xyz.setX(0.0);
	new_target_xyz.setY(0.0);
	new_target_xyz.setZ(0.0);
      }
      
      geometry_msgs::Vector3Stamped tar_pos_msg;
      tar_pos_msg.header = msg->header;
      tar_pos_msg.vector = tf2::toMsg(cam_tf * new_target_xyz);
      
      aerial_robot_msgs::FlightNav msg;
      
      msg.pos_xy_nav_mode = msg.POS_MODE;
      msg.psi_nav_mode = msg.NO_NAVIGATION;
      msg.pos_z_nav_mode = msg.NO_NAVIGATION;
      msg.header.stamp = ros::Time(0);
      msg.control_frame = 0;
      
      // msg.target_pos_x = (tar_pos_msg.vector.x + camera_base_translation_world_msg.vector.x) * coef_x;
      // msg.target_pos_y = (tar_pos_msg.vector.y + camera_base_translation_world_msg.vector.y) * coef_y;
      // msg.target_pos_z = (tar_pos_msg.vector.z + camera_base_translation_world_msg.vector.z) * coef_z;

      // msg.target_pos_x = (tar_pos_msg.vector.x  * coef_x) + camera_base_translation_world_msg.vector.x;
      // msg.target_pos_y = (tar_pos_msg.vector.y  * coef_y) + camera_base_translation_world_msg.vector.y;
      // msg.target_pos_z = (tar_pos_msg.vector.z  * coef_z) + camera_base_translation_world_msg.vector.z;

      msg.target_pos_x = (tar_pos_msg.vector.x * 1.0) + camera_base_translation.x();
      msg.target_pos_y = (tar_pos_msg.vector.y * 1.0) + camera_base_translation.y();
      msg.target_pos_z = (tar_pos_msg.vector.z * 1.0) + camera_base_translation.z();

      // std::cout << camera_base_translation_world_msg.vector.x << std::endl;
      // std::cout << camera_base_translation_world_msg.vector.y << std::endl;
      // std::cout << camera_base_translation_world_msg.vector.z << std::endl;

      std::cout << camera_base_translation.x() << std::endl;
      std::cout << camera_base_translation.y() << std::endl;
      std::cout << camera_base_translation.z() << std::endl;

      target_pos_pub.publish(msg);
      std::this_thread::sleep_for(std::chrono::seconds(5));
      
      std::cout << "target_x : "<< msg.target_pos_x << std::endl;
      std::cout << "target_y : "<< msg.target_pos_y << std::endl;
      std::cout << "target_z : "<< msg.target_pos_z << std::endl;
      
      std::cout << "working_phase : 6" << std::endl;
      std::cout << " " << std::endl;
      
      working_phase = 5;
    }


  }

  void RedObjectDetectionWithHSVFilter::imagedepthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(working_phase == 0 && working_phase_py == 0){

      working_phase = 1;
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv::Mat src_image = cv_ptr->image;
      cv::imwrite("/home/kuromiya/depth_org.png", src_image);
      cv::Mat mask = cv::Mat(src_image != src_image);
      cv::normalize(src_image, src_image, 255, 0, cv::NORM_MINMAX);
      depth_img = src_image;
      cv::imwrite("/home/kuromiya/depth.png", depth_img);
      working_phase = 2;
    }
  }

  void RedObjectDetectionWithHSVFilter::workingphaseCallback(const std_msgs::Int32 msg)
  {
    // working_phase_py = msg.data;
  }

  void RedObjectDetectionWithHSVFilter::planeCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", ros::Time(0));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    
    if(working_phase == 2){
      geometry_msgs::PolygonStamped polygons_stamped[sizeof(msg->polygons) / sizeof(msg->polygons[0])];
      geometry_msgs::Polygon image_plane;

      tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
      
      double roll, pitch, yaw;
      cam_tf_rotation.getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
      cam_angle = yaw;
      
      geometry_msgs::Vector3Stamped obj_pos;
      double highest_x = 0.0;
      double highest_y = 0.0;
      std::vector<double> highest_z;
      tf2::Vector3 highest_normal_world;

      for(const auto polygon_stamped : msg->polygons)
        {
          geometry_msgs::Polygon polygon = polygon_stamped.polygon;
          std::vector<int> img_point;  // store points of a plane in image coordinate

          for(const auto point : polygon.points)
            {
              tf2::Vector3 plane;
              plane.setX(point.x);
              plane.setY(point.y);
              plane.setZ(point.z);

            }


          jsk_recognition_utils::Vertices vertice;
          for (size_t i = 0; i < polygon.points.size(); i++) {
            Eigen::Vector3f v;
            jsk_recognition_utils::pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(polygon.points[i], v);
            vertice.push_back(v);
          }
          jsk_recognition_utils::Polygon area_polygon(vertice);
          Eigen::Vector3f normal_cam = area_polygon.getNormalFromVertices();
          tf2::Vector3 normal;
          normal.setX(normal_cam(0));
          normal.setY(normal_cam(1));
          normal.setZ(normal_cam(2));
          tf2::Vector3 normal_world = cam_tf_rotation * normal;

          Eigen::Vector3f center_cam = area_polygon.centroid();
          tf2::Vector3 center;
          center.setX(center_cam(0));
          center.setY(center_cam(1));
          center.setZ(center_cam(2));
          area_polygon.fromROSMsg(polygon);
          std_msgs::Float64 area;
          area.data = area_polygon.area();
          area_pub.publish(area);

          geometry_msgs::TransformStamped obj_tf_msg;
          obj_tf_msg.header = msg->header;
          obj_tf_msg.header.frame_id = std::string("world");
          obj_tf_msg.child_frame_id = frame_id_;
          obj_tf_msg.transform.translation = tf2::toMsg(cam_tf * center);
          obj_tf_msg.transform.rotation.w = 1.0;
          tf_br_.sendTransform(obj_tf_msg);

          geometry_msgs::Vector3Stamped obj_pos_msg;
          obj_pos_msg.header = obj_tf_msg.header;
          obj_pos_msg.vector = obj_tf_msg.transform.translation;
	
          tf2::Vector3 center_world = cam_tf * center;

          // store 5 vetice candidates

          // judge whether the detected plane is truly one of the surface of the target objects
          if(std::abs(area.data - 0.040) < 0.010 || std::abs(area.data - 0.060) < 0.010){
            // detect only the upper surface
            if(std::abs(normal_world.z()) > 0.8){  //  change < 0.1 when ground mode
              if(vertice_highest.size() < 5){
                vertice_highest.push_back(vertice);
                highest_z.push_back(center_world.z());
	      }
              else{
              int z_count = 0;
              for(const auto z : highest_z){
                if(center_world.z() > z){
                  highest_z.erase(highest_z.begin() + z_count);
                  vertice_highest.erase(vertice_highest.begin() + z_count);
                  highest_z.push_back(center_world.z());
                  vertice_highest.push_back(vertice);
                  break;
                }
                z_count++;
              }
              }
            }
          }
        }
    working_phase = 3;
    }

    if(working_phase == 5){
      geometry_msgs::PolygonStamped polygons_stamped[sizeof(msg->polygons) / sizeof(msg->polygons[0])];
      geometry_msgs::Polygon image_plane;

      tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
      
      double roll, pitch, yaw;
      cam_tf_rotation.getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
      cam_angle = yaw;
      
      geometry_msgs::Vector3Stamped obj_pos;
      double highest_x = 0.0;
      double highest_y = 0.0;
      std::vector<double> highest_z;
      tf2::Vector3 highest_normal_world;

      for(const auto polygon_stamped : msg->polygons)
        {
          geometry_msgs::Polygon polygon = polygon_stamped.polygon;

          jsk_recognition_utils::Vertices vertice;
          for (size_t i = 0; i < polygon.points.size(); i++) {
            Eigen::Vector3f v;
            jsk_recognition_utils::pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(polygon.points[i], v);
            vertice.push_back(v);
          }
          jsk_recognition_utils::Polygon area_polygon(vertice);
          Eigen::Vector3f normal_cam = area_polygon.getNormalFromVertices();
          tf2::Vector3 normal;
          normal.setX(normal_cam(0));
          normal.setY(normal_cam(1));
          normal.setZ(normal_cam(2));
          tf2::Vector3 normal_world = cam_tf_rotation * normal;

          Eigen::Vector3f center_cam = area_polygon.centroid();
	  
	  tf2::Vector3 center;

	  center.setX(center_cam(0));
	  center.setY(center_cam(1));
	  center.setZ(center_cam(2));

	  tf2::Vector3 center_world = cam_tf * center;

	  std::cout << "world target_x : " << center_world.x() << std::endl;
	  std::cout << "world target_y : " << center_world.y() << std::endl;
	  std::cout << "world target_z : " << center_world.z() << std::endl;
	  
	  if(normal_world.z() < 0.1 && center_world.z() > 0.3){
	  
	    cam_target_xyz = center;

	    std::cout << "Approach" << std::endl;

	    std::cout << "working_phase : 5" << std::endl;
	    working_phase = 6;

	    break;
	  }

	  std::cout << "working_phase : 5" << std::endl;
	  working_phase = 6;
	}
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc2020_task2_tasks::RedObjectDetectionWithHSVFilter, nodelet::Nodelet);
