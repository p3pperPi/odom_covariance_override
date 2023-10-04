#include "odom_covariance_override/odom_covariance_override.hpp"


#define DEBUG_ON



OdomCovarianceOverride::OdomCovarianceOverride()
: Node("odom_covariance_override")
{
  // read parameters
  this->declare_parameter("subscribe_topic_name", "odometry/gps/raw");
  this->declare_parameter("publish_topic_name", "odom");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("pose_covariance_magnification", 0.5);
  this->declare_parameter("distance_threshold", 0.1);

  std::string sub_topic_name = this->get_parameter("subscribe_topic_name").as_string();
  std::string pub_topic_name = this->get_parameter("publish_topic_name").as_string();

  // pub/sub initialize
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_topic_name, 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    sub_topic_name, 10, std::bind(&OdomCovarianceOverride::odom_callback, this, _1));
}

void OdomCovarianceOverride::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  prev_time = recv_time;
  prev_pose = recv_pose;
  prev_pose_cov = recv_pose_cov;

  recv_time.sec = odom -> header.stamp.sec;
  recv_time.nanosec = odom -> header.stamp.nanosec;

  #ifdef DEBUG_ON
    RCLCPP_INFO(this->get_logger(), "recv_time [%u.%u]", recv_time.sec, recv_time.nanosec);
  #endif

  recv_pose = odom -> pose.pose;
  recv_pose_cov = odom -> pose.covariance;

  calc_vel_theta();
}

void OdomCovarianceOverride::publish()
{
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);
  std::string frame_id = this->get_parameter("frame_id").as_string();

  pose_yaw_covariance = calc_yaw_covariance();

  pose_cov = {
    static_cast<double>(recv_pose_cov[0]), 0., 0., 0., 0., 0.,
    0., static_cast<double>(recv_pose_cov[7]), 0., 0., 0., 0.,
    0., 0., static_cast<double>(recv_pose_cov[14]), 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., pose_yaw_covariance };


  // set the header
  odom.header.stamp.sec = recv_time.sec;
  odom.header.stamp.nanosec = recv_time.nanosec;
  odom.header.frame_id = frame_id;

  // set the position
  odom.pose.pose.position = recv_pose.position;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = pose_cov;

  // set the velocity
  odom.twist.twist.linear.x = velocity;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;


  odom_pub_ -> publish(odom);
}

void OdomCovarianceOverride::calc_vel_theta()
{
  geometry_msgs::msg::Pose pose_distance;
  double distance, dt, rad;

  if(!prev_pose.position.x || !prev_pose.position.y) return;

  pose_distance.position.x = recv_pose.position.x - prev_pose.position.x;
  pose_distance.position.y = recv_pose.position.y - prev_pose.position.y;

  distance = pow(pose_distance.position.x, 2.0) + pow(pose_distance.position.y, 2.0);
  distance = sqrt(distance);

  dt = (recv_time.sec + (double)recv_time.nanosec / 1000000000) - (prev_time.sec + (double)prev_time.nanosec / 1000000000);
  velocity = (dt * distance) * (1 / dt);

  rad = atan2(recv_pose.position.y - prev_pose.position.y, recv_pose.position.x - prev_pose.position.x);

  quat.setRPY(0,0,rad);
  quat = quat.normalize();


  #ifdef DEBUG_ON
    RCLCPP_INFO(this->get_logger(), "distance [%lf]", distance);
    RCLCPP_INFO(this->get_logger(), "dt [%f]", dt);
    RCLCPP_INFO(this->get_logger(), "x velocity [%f]", velocity);
    RCLCPP_INFO(this->get_logger(), "quaternion [%f, %f, %f, %f]", quat.getX(), quat.getY(), quat.getZ(), quat.getW());
  #endif

  publish();
}



/*
* yaw角度のcovarianceを計算します。
*
* @return 現在座標と前回座標のcovarianceから取りうる角度のcovarianceを求めて、返します。asin関数の範囲外の場合、covarianceは2pi^2を返し、yaw角度の信頼度を限りなくゼロにします。
*/
double OdomCovarianceOverride::calc_yaw_covariance()
{
  double stheta_t, distance, cur_sx, prev_sx;
  double pc_mag = this->get_parameter("pose_covariance_magnification").as_double();
  double distance_threshold = this->get_parameter("distance_threshold").as_double();

  distance = pow((recv_pose.position.x - prev_pose.position.x), 2.0) + pow((recv_pose.position.y - prev_pose.position.y), 2.0);
  distance = sqrt(distance);

  if(distance > distance_threshold)
  {
      cur_sx = sqrt(recv_pose_cov[0]) * pc_mag;
      prev_sx = sqrt(prev_pose_cov[0]) * pc_mag;

      stheta_t = (cur_sx + prev_sx) / distance;

      errno = 0;  // asinエラーハンドリング用
      stheta_t = asin(stheta_t);

      pose_yaw_covariance = (errno == 0) ? pow(stheta_t, 2.0) :  39.4784176043574;  // 範囲外の場合は2pi^2
  }
  else
  {
    pose_yaw_covariance = 39.4784176043574; // 2pi^2
  }

  #ifdef DEBUG_ON
    RCLCPP_INFO(this->get_logger(), "yaw_cov= [%f]", stheta_t);
  #endif


  return pose_yaw_covariance;
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<OdomCovarianceOverride> node = std::make_shared<OdomCovarianceOverride>();

  try
  {
    while(rclcpp::ok())
    {
      rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
  }
  catch(const rclcpp::exceptions::RCLError &e)
  {
    RCLCPP_ERROR(node->get_logger(), "unexpectedly failed with %s ", e.what());
    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "std::exception error occured: %s ", e.what());
    rclcpp::shutdown();
  }

  return 0;
}