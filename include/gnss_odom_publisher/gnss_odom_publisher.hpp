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
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void publish();
    public:
        GnssOdomPublisher();
};

#endif