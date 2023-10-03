#include "gnss_odom_publisher/gnss_odom_publisher.hpp"


#define DEBUG_ON



GnssOdomPublisher::GnssOdomPublisher()
: Node("gnss_odom_publisher")
{
  // read parameters
  // 今はべた書き
  sub_topic_name = "odometry/gps/raw";
  pub_topic_name = "odom";
  frame_id = "map";


  // pub/sub initialize
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_topic_name, 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    sub_topic_name, 10, std::bind(&GnssOdomPublisher::odom_callback, this, _1));
}

void GnssOdomPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  prev_time = recv_time;
  prev_pose = recv_pose;

  recv_time.sec = odom -> header.stamp.sec;
  recv_time.nanosec = odom -> header.stamp.nanosec;

  #ifdef DEBUG_ON
    RCLCPP_INFO(this->get_logger(), "recv_time [%u.%u]", recv_time.sec, recv_time.nanosec);
  #endif

  recv_pose = odom -> pose.pose;
  recv_pose_cov = odom -> pose.covariance;

  calc_vel_theta();
}

void GnssOdomPublisher::publish()
{
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);

  pose_yaw_covariance = 0.5;

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

void GnssOdomPublisher::calc_vel_theta()
{
  geometry_msgs::msg::Pose pose_distance;
  double distance, dt, rad;

  if(!prev_pose.position.x || !prev_pose.position.y) return;

  pose_distance.position.x = recv_pose.position.x - prev_pose.position.x;
  pose_distance.position.y = recv_pose.position.y - prev_pose.position.y;

  distance = pow(pose_distance.position.x, 2.0) + pow(pose_distance.position.y, 2.0);
  distance = pow(distance, 0.5);

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




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<GnssOdomPublisher> node = std::make_shared<GnssOdomPublisher>();

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