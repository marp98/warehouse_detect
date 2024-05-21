#include "my_components/preapproach_component.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: Node("preapproach_node", options), rotation_degrees_(-90.0), start_pose_set_(false)
{
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    auto laser_callback = [this](const typename sensor_msgs::msg::LaserScan::SharedPtr msg) -> void {
        if (!is_rotating_) {
            if (msg->ranges[msg->ranges.size() / 2] <= obstacle_distance_) {
                stopRobot();
                is_rotating_ = true;
                RCLCPP_INFO(this->get_logger(), "Obstacle detected");
            } else {
                moveRobot();
            }
        }
    };
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, laser_callback);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));
}

void PreApproach::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (is_rotating_) {
        if (!start_pose_set_) {
            start_pose_ = msg->pose.pose;
            start_pose_set_ = true;
            setUpRotate();
        }

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double yaw_tolerance = 0.007;

        if (std::abs(yaw - target_yaw_) < yaw_tolerance) {
            stopRobot();
            is_rotating_ = false;
            RCLCPP_INFO(this->get_logger(), "Rotation completed. Stopping the robot.");
            RCLCPP_INFO(this->get_logger(), "Final Yaw: %.2f", yaw);

            laser_sub_.reset();
        } else {
            rotateRobot();
        }
    }
}

void PreApproach::setUpRotate()
{
    double target_yaw = rotation_degrees_ * M_PI / 180.0;
    target_yaw_ = target_yaw;

    RCLCPP_INFO(this->get_logger(), "Target Yaw: %.2f", target_yaw_);
}

void PreApproach::rotateRobot()
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.2 * std::copysign(1.0, rotation_degrees_);
    cmd_vel_pub_->publish(twist);
}

void PreApproach::moveRobot()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.4;
    cmd_vel_pub_->publish(cmd_vel);
}

void PreApproach::stopRobot()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Stopping the robot");
}
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)