#ifndef GNSS_ODOM_PUBLISHER_H
#define GNSS_ODOM_PUBLISHER_H


// ros2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"



// cpp
#include <memory>
#include <math.h>



using std::placeholders::_1;


class GnssOdomPublisher : public rclcpp::Node
{
    private:
        rclcpp::Time subscribe_time;    // 一旦仮

        rclcpp::Time recv_time;
        geometry_msgs::msg::Pose recv_pose;
        std::array<double, 36> recv_cov;

        rclcpp::Time prev_time;
        geometry_msgs::msg::Pose prev_pose;

        geometry_msgs::msg::Pose current_pose;
        geometry_msgs::msg::Twist current_twist;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void publish();
        void calc_vel_theta();
    public:
        GnssOdomPublisher();
};

#endif