#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class GnssOdomPublisher : public rclcpp::Node
{
  public:
    GnssOdomPublisher()
    : Node("gnss_odom_publisher")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps/raw", 10, std::bind(&GnssOdomPublisher::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Position-> x: [%f]", msg->pose.pose.position.x);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}