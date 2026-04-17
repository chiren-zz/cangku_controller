#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <cmath>

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher();

private:
    void handle_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 校准参数
    double linear_scale_x_ = 1.0;
    double linear_scale_y_ = 1.0;
    double angular_scale_  = 1.0;

    // 积分状态
    double x_pos_   = 0.0;
    double y_pos_   = 0.0;
    double heading_ = 0.0;
    double vel_dt_  = 0.0;

    // 当前速度
    double linear_velocity_x_ = 0.0;
    double linear_velocity_y_ = 0.0;
    double angular_velocity_z_ = 0.0;

    // 参数
    double      wheelbase_           = 0.25;
    bool        pub_odom_tf_         = false;
    bool        first_vel_           = true;
    std::string odom_frame_          = "odom";
    std::string base_footprint_frame_ = "base_footprint";

    rclcpp::Time last_vel_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>             tf_broadcaster_;
};