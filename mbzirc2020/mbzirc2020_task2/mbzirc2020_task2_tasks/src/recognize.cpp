#include <mbzirc2020_task2_tasks/recognize.h>
#include <mbzirc2020_task2_tasks/kmeans.h>


namespace mbzirc2020_task2_tasks
{
    if(real_size_scale_ == 0) return; // no receive camera_info yet.

    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", ros::Time(0));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat src_image = cv_ptr->image;

    // detect the target objects

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
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc2020_task2_tasks::RedObjectDetectionWithHSVFilter, nodelet::Nodelet);
