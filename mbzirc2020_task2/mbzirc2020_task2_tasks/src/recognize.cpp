#include </home/kuromiya/ros/aerial_robot_demo_ws/src/aerial_robot_demo/mbzirc2020_task2/mbzirc2020_task2_tasks/include/recognize.h>
#include </home/kuromiya/ros/aerial_robot_demo_ws/src/aerial_robot_demo/mbzirc2020_task2/mbzirc2020_task2_tasks/include/kmeans.cpp>
#include </home/kuromiya/ros/aerial_robot_demo_ws/src/aerial_robot_demo/mbzirc2020_task2/mbzirc2020_task2_tasks/include/space_detector.cpp>

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

    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/pos"), 1);
    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    xy_pub = advertise<geometry_msgs::Point>(*nh_, "xy_point", 1);
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);
    area_pub = advertise<std_msgs::Float64>(*nh_, "area", 1);
    marker_pub = advertise<visualization_msgs::Marker>(*nh_, "marker", 1);
    angle_pub = advertise<std_msgs::Float64>(*nh_, "target_object/angle", 1);

    working_fhase = 1;

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void RedObjectDetectionWithHSVFilter::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &RedObjectDetectionWithHSVFilter::imageCallback, this);
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &RedObjectDetectionWithHSVFilter::cameraInfoCallback, this);
    plane_sub_ = nh_->subscribe("polygon_array", 1, &RedObjectDetectionWithHSVFilter::planeCallback, this);
    image_depth_sub_ = it_->subscribe("image_depth", 1, &RedObjectDetectionWithHSVFilter::imagedepthCallback, this);
  }

  void RedObjectDetectionWithHSVFilter::unsubscribe()
  {
    image_sub_.shutdown();
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
    object_distance = 15.0;  // change until you can get an optimal height
    camdep.setCameraInfo(*msg);
    cam_info_sub_.shutdown();
  }

  void RedObjectDetectionWithHSVFilter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(real_size_scale_ == 0) return; // no receive camera_info yet.

    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_camera_optical_frame", msg->header.stamp);
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf2::Transform leg_cog_tf;
    try{
      geometry_msgs::TransformStamped leg_cog_msg = tf_buff_.lookupTransform("cog", "leg5", msg->header.stamp);
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

    if(working_fhase == 0){
      cv::Mat hsv_image;
      cvtColor(src_image, hsv_image, cv::COLOR_BGR2HSV);

      cv::Mat mask = cv::Mat::zeros(src_image.rows, src_image.cols, CV_8UC3);

      DataFrame data;
      int i_count = 0;  //  highest point
      geometry_msgs::Point xy_center;

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

      // contour approximation
      cv::Mat graymask;
      cv::cvtColor(mask, graymask, cv::COLOR_BGR2GRAY);
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(graymask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

      double max_contour_area = 0;
      std::vector<cv::Point> target_contour;

      auto calc_position = [](std::vector<cv::Point> contour) {
                             cv::Moments contour_moments = cv::moments(contour, true);
                             tf2::Vector3 pos;
                             pos.setX(contour_moments.m10 / contour_moments.m00);
                             pos.setY(contour_moments.m01 / contour_moments.m00);
                             return pos;
                           };

      for(const auto& contour : contours) {
        double real_contour_area = cv::contourArea(contour);
        NODELET_DEBUG_STREAM("contour size" << real_contour_area);

        if(true) {
          /* find maximum size */
          if (max_contour_area < real_contour_area) {
            max_contour_area = real_contour_area;
            target_contour = contour;
          }
        }
      }

      tf2::Vector3 target_obj_uv = calc_position(target_contour);

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
      target_pos_pub_.publish(obj_pos_msg);

      image_pub_.publish(cv_ptr->toImageMsg());
      usleep(130 * 1000);  // wait for 50s 
      working_fhase = 1;
    }

    // draw plane points(maybe not necessarry)
    // if(planes_img.size() != 0){
    //   for(const auto plane_img : planes_img){
    //     int x = 0;
    //     int y = 0;
    //     for(int i = 0; i != plane_img.size(); i++){
    //       if(i % 2 == 0){
    //         x = plane_img[i];
    //       }
    //       else{
    //         y = plane_img[i];
    //       }
    //     }
    //   }
    //   // image_pub_.publish(cv_ptr->toImageMsg());
    //   planes_img.erase(planes_img.begin(), planes_img.end());
    // }

    // draw polygon line
    if(working_fhase == 3){
      int count = 0;
      if(vertice_highest.size() != 0){
        // std::cout << "size : " << vertice_highest.size() << std::endl;
        for(const auto v_highest : vertice_highest){
        jsk_recognition_utils::Polygon line_img_polygon(v_highest);
        cv::Mat canvas(src_image.rows, src_image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
        line_img_polygon.drawLineToImage(camdep, cv_ptr->image, cv::Scalar(0, 255, 0), 1);
        line_img_polygon.drawLineToImage(camdep, canvas, cv::Scalar(0, 255, 0), 1);
        if(count == 0){
          cv::imwrite("/home/kuromiya/draw_plane.png", canvas);
        }

        if(depth_img.cols != 0 && depth_img.rows != 0){
          std::vector<float> answer = space_detector(canvas, depth_img);
          // std::cout << "answer[1] : " << answer[1] << std::endl;
          if(answer[1] == 0){
            continue;
          }
          else if(answer[1] == 1){
            float x = line_img_polygon.centroid()(0);
            float y = line_img_polygon.centroid()(1);
            float z = line_img_polygon.centroid()(2);

            tf2::Vector3 target_xyz;
            target_xyz.setX(x+leg_cog_translation.x());
            target_xyz.setY(y+leg_cog_translation.y());
            target_xyz.setZ(z);
            geometry_msgs::Vector3Stamped tar_pos_msg;
            tar_pos_msg.header = msg->header;
            tar_pos_msg.vector = tf2::toMsg(cam_tf * target_xyz);
            target_pos_pub_.publish(tar_pos_msg);
            std_msgs::Float64 angle;
            angle.data = answer[0] * 3.14 / 180;
            angle_pub.publish(angle);
            answer.erase(answer.begin(), answer.end());
            // usleep(30 * 1000);  // wait for 30s
          }
          image_pub_.publish(cv_ptr->toImageMsg());
          count++;
        }
        }
        // std::cout << "OK" << std::endl;
      }
       vertice_highest.erase(vertice_highest.begin(), vertice_highest.end());
       working_fhase = 1;
    }
  }

  void RedObjectDetectionWithHSVFilter::imagedepthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(working_fhase == 1){
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv::Mat src_image = cv_ptr->image;
      cv::imwrite("/home/kuromiya/depth_org.png", src_image);
      cv::Mat mask = cv::Mat(src_image != src_image);
      cv::normalize(src_image, src_image, 255, 0, cv::NORM_MINMAX);
      depth_img = src_image;
      cv::imwrite("/home/kuromiya/depth.png", depth_img);
      working_fhase = 2;
    }
  }

  void RedObjectDetectionWithHSVFilter::planeCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    if(working_fhase == 2){
      geometry_msgs::PolygonStamped polygons_stamped[sizeof(msg->polygons) / sizeof(msg->polygons[0])];
      geometry_msgs::Polygon image_plane;

      tf2::Transform cam_tf;
      try{
        geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_camera_optical_frame", msg->header.stamp);
        tf2::convert(cam_pose_msg.transform, cam_tf);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }

      tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
      geometry_msgs::Vector3Stamped obj_pos;
      double highest_x = 0.0;
      double highest_y = 0.0;
      std::vector<double> highest_z;
      tf2::Vector3 highest_normal_world;

      for(const auto polygon_stamped : msg->polygons)
        {
          geometry_msgs::Polygon polygon = polygon_stamped.polygon;
          std::vector<int> img_point;  // store points of a plane on image

          for(const auto point : polygon.points)
            {
              tf2::Vector3 plane;
              plane.setX(point.x);
              plane.setY(point.y);
              plane.setZ(point.z);

              // tf2::Vector3 a_img_point = camera_K * plane / point.z;
              // img_point.push_back(a_img_point.x());
              // img_point.push_back(a_img_point.y());
            }

          // planes_img.push_back(img_point);
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
          // obj_pos_msg.vector = tf2::toMsg(cam_tf_rotation * normal);
          // target_pos_pub_.publish(obj_pos_msg);

          tf2::Vector3 center_world = cam_tf * center;

          // store 5 vetice candidates

          if(std::abs(area.data - 0.040) < 0.010 || std::abs(area.data - 0.060) < 0.010){
            if(std::abs(normal_world.z()) > 0.8){
              if(vertice_highest.size() < 5){
                vertice_highest.push_back(vertice);
                highest_z.push_back(center_world.z());
              }
              else{
              int z_count = 0;
              for(const auto z : highest_z){
                if(center_world.z() > z){
                  // obj_pos.header = obj_tf_msg.header;
                  // obj_pos.vector = tf2::toMsg(cam_tf * center);
                  // highest_x = center_world.x();
                  // highest_y = center_world.y();
                  // highest_z = center_world.z();
                  // highest_normal_world = normal_world;
                  highest_z.erase(highest_z.begin() + z_count);
                  vertice_highest.erase(vertice_highest.begin() + z_count);
                  highest_z.push_back(center_world.z());
                  vertice_highest.push_back(vertice);
                  break;
                  // vertice_highest = vertice;
                }
                z_count++;
              }
              }
            }
          }
        }

      // visualization_msgs::Marker marker;
      // marker.header.frame_id = "/world";
      // marker.header.stamp = ros::Time::now();
      // marker.ns = "basic_shapes";
      // marker.id = 0;

      // marker.type = visualization_msgs::Marker::CUBE;
      // marker.action = visualization_msgs::Marker::ADD;
      // marker.lifetime = ros::Duration();

      // marker.scale.x = 0.2;
      // marker.scale.y = 0.2;
      // marker.scale.z = 0.3;
      // marker.pose.position.x = highest_x - marker.scale.x / 2 * highest_normal_world.x();
      // marker.pose.position.y = highest_y - marker.scale.y / 2 * highest_normal_world.y();
      // marker.pose.position.z = highest_z - marker.scale.z / 2 * highest_normal_world.z();
      // marker.pose.orientation.x = 0.0;
      // marker.pose.orientation.y = 0.0;
      // marker.pose.orientation.z = 0.496477869;
      // marker.pose.orientation.w = 0.868049379;
      // marker.color.r = 0.0f;
      // marker.color.g = 1.0f;
      // marker.color.b = 0.0f;
      // marker.color.a = 1.0f;
      // marker_pub.publish(marker);
      // target_pos_pub_.publish(obj_pos);
    }
    working_fhase = 3;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc2020_task2_tasks::RedObjectDetectionWithHSVFilter, nodelet::Nodelet);
