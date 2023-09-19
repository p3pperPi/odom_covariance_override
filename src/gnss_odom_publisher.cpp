#include "gnss_odom_publisher/gnss_odom_publisher.hpp"





GnssOdomPublisher::GnssOdomPublisher()
: Node("gnss_odom_publisher")
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/gps/raw", 10, std::bind(&GnssOdomPublisher::odom_callback, this, _1));

  // read parameters
}

void GnssOdomPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Position-> x: [%f]", msg->pose.pose.position.x);
}

void GnssOdomPublisher::publish()
{
  return;
}