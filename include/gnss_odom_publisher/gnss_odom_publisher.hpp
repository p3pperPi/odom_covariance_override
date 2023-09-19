#ifndef GNSS_ODOM_PUBLISHER_H
#define GNSS_ODOM_PUBLISHER_H


// ros2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"



// cpp
#include <memory>



using std::placeholders::_1;


class GnssOdomPublisher : public rclcpp::Node
{
    private:
        rclcpp::Time subscribe_time;    // 一旦仮

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void publish();
    public:
        GnssOdomPublisher();
};

#endif