#include "gnss_odom_publisher/gnss_odom_publisher.hpp"





GnssOdomPublisher::GnssOdomPublisher()
: Node("gnss_odom_publisher")
{
  // read parameters


  // pub/sub initialize
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/gps/raw", 10, std::bind(&GnssOdomPublisher::odom_callback, this, _1));
}

void GnssOdomPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  //RCLCPP_INFO(this->get_logger(), "Position-> x: [%f]", odom->pose.pose.position.x);
  subscribe_time = this -> get_clock() -> now();
  publish();
}

void GnssOdomPublisher::publish()
{
  nav_msgs::msg::Odometry odom;

  /* 仮で適当な値をpublishさせる */

  odom.header.stamp = subscribe_time;
  odom.header.frame_id = std::string("odom_frame");
  odom.child_frame_id = std::string("odom_child_frame");
  // set the position
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  // set the velocity
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  odom_pub_ -> publish(odom);
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

  return 0;
}