#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class TargetPoseConvert
{
public:
  TargetPoseConvert(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp),
    once_flag_(false)
  {
    nhp_.param("fixed_transform_flag", fixed_transform_flag_, false);
    nhp_.param("baselink_frame", baselink_frame_, std::string("baselink"));

    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);
    target_world_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target_pos_world", 1);
    uav_odom_sub_ = nh_.subscribe("uav_odom", 1, &TargetPoseConvert::odomCallback, this);
    target_local_pos_sub_ = nh_.subscribe("target_local_pos", 1, &TargetPoseConvert::targetLocalPosCallback, this);
  }

private:
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber uav_odom_sub_;
  ros::Subscriber target_local_pos_sub_;
  ros::Publisher target_world_pos_pub_;

  tf2_ros::Buffer tf_buff_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

  std::string baselink_frame_;
  bool fixed_transform_flag_;
  bool once_flag_;
  tf2::Transform baselink2sensor_tf_;
  tf2::Transform world2baselink_tf_;

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    tf2::convert(msg->pose.pose, world2baselink_tf_);
  }

  void targetLocalPosCallback(const geometry_msgs::PointStampedConstPtr& msg)
  {
    if(!once_flag_)
      {
        try
          {
            geometry_msgs::TransformStamped baselink2sensor_msg = tf_buff_.lookupTransform(baselink_frame_, msg->header.frame_id, msg->header.stamp);
            tf2::convert(baselink2sensor_msg.transform, baselink2sensor_tf_);

            if(fixed_transform_flag_) once_flag_ = true;
          }
        catch (tf2::TransformException &ex)
          {
            ROS_WARN("%s",ex.what());
            return;
          }
      }

    tf2::Vector3 target_local_pos;
    tf2::fromMsg(msg->point, target_local_pos);

    geometry_msgs::PointStamped target_world_pos_msg;
    target_world_pos_msg.header = msg->header;
    target_world_pos_msg.header.frame_id = "world";
    tf2::toMsg(world2baselink_tf_ * baselink2sensor_tf_ * target_local_pos, target_world_pos_msg.point);
    target_world_pos_pub_.publish(target_world_pos_msg);
  }

};


int main (int argc, char **argv)
{
  ros::init (argc, argv, "target_pose_converter");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  TargetPoseConvert target_pose_convert(nh, nhp);

  ros::spin();

  return 0;
}

