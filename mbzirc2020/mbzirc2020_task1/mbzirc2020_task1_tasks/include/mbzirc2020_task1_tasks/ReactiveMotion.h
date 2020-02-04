#include <mbzirc2020_task1_tasks/RansacLineFitting.h>

#include <ros/ros.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

class ReactiveMotion{
#define STILL 0
#define WAITING 1
#define TRACKING 2
#define STOP_TRACKING 3
#define LOSING_TRACKING 4
public:
  ReactiveMotion(ros::NodeHandle nh, ros::NodeHandle nhp);

private:
  /* basic */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  RansacLineFitting *ransac_line_estimator_;
  nav_msgs::Odometry cog_odom_;
  double control_freq_;
  int motion_state_;
  Eigen::Vector3d cur_pos_;
  Eigen::Vector3d target_pos_;
  Eigen::Vector3d euler_ang_;
  int stop_tracking_cnt_;
  double cog_net_offset_;
  double target_pos_xy_thre_;
  std::vector<double> motion_cmd_thre_vec_;

  Eigen::Vector3d task_initial_waiting_pos_;
  bool task_initial_waiting_pos_flag_;
  double experiment_safety_z_offset_;
  int losing_tracking_cnt_;
  double losing_tracking_period_thre_;
  int task_track_flag_pub_cnt_;

  ros::Timer control_timer_;
  ros::Subscriber cog_odom_sub_;
  ros::Subscriber motion_state_sub_;
  ros::Publisher flight_nav_pub_;
  ros::Publisher nearest_waypoint_pub_;
  ros::Publisher uav_cog_point_pub_;
  ros::Publisher task_return_initial_waypt_pub_;
  ros::Publisher task_track_flag_pub_;

  void sendControlCmd(Eigen::Vector3d target_pos);
  bool isTargetPosInSearchRegion();

  void controlTimerCallback(const ros::TimerEvent& event);
  void cogOdomCallback(const nav_msgs::OdometryConstPtr & msg);
  void reactiveMotionStateCallback(const std_msgs::Int8ConstPtr & msg);
};
