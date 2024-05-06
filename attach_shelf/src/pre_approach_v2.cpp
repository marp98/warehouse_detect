#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include "approach_shelf_msg/srv/go_to_loading.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;
using GoToLoading = approach_shelf_msg::srv::GoToLoading;

class PreApproach : public rclcpp::Node {
public:
    PreApproach() : Node("pre_approach"), is_rotating_(false), is_stopped_(false), start_pose_set_(false), final_approach_performed_(false)  {
        this->declare_parameter("obstacle");
        this->declare_parameter("degrees");
        this->declare_parameter("final_approach");
        obstacle_distance_ = this->get_parameter("obstacle").as_double();
        rotation_degrees_ = this->get_parameter("degrees").as_int();
        final_approach_ = this->get_parameter("final_approach").as_bool();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        approach_shelf_client_ = this->create_client<GoToLoading>("approach_shelf");
    }

private:
    double obstacle_distance_;
    int rotation_degrees_;
    bool final_approach_;
    bool is_rotating_;
    bool is_stopped_;
    bool start_pose_set_;
    double start_yaw_;
    double target_yaw_;
    bool final_approach_performed_;

    geometry_msgs::msg::Pose start_pose_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<GoToLoading>::SharedPtr approach_shelf_client_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (is_rotating_ || is_stopped_) {
            return;
        }

        double front_range = msg->ranges[msg->ranges.size() / 2];

        geometry_msgs::msg::Twist twist;
        if (front_range > obstacle_distance_) {
            twist.linear.x = 0.4;
            twist.angular.z = 0.0;
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            is_rotating_ = true;
        }
        cmd_vel_pub_->publish(twist);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!is_rotating_ || is_stopped_) {
            return;
        }

        if (is_rotating_ && !start_pose_set_) {
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

        if (std::abs(yaw - target_yaw_) < 0.1) {
            geometry_msgs::msg::Twist stop_twist;
            stop_twist.linear.x = 0.0;
            stop_twist.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_twist);

            is_rotating_ = false;
            is_stopped_ = true;

            if (!final_approach_performed_) {
                performFinalApproach();
                final_approach_performed_ = true;
            }
        } else {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.2 * std::copysign(1.0, rotation_degrees_);
            cmd_vel_pub_->publish(twist);
        }
    }

    void setUpRotate() {
        tf2::Quaternion q(
            start_pose_.orientation.x,
            start_pose_.orientation.y,
            start_pose_.orientation.z,
            start_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        start_yaw_ = yaw;
        target_yaw_ = start_yaw_ + rotation_degrees_ * M_PI / 180.0;
    }

    void performFinalApproach() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Performing final approach");
        if (final_approach_) {
                while (!approach_shelf_client_->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                }

                auto request = std::make_shared<GoToLoading::Request>();
                request->attach_to_shelf = final_approach_;

                auto result_future = approach_shelf_client_->async_send_request(request);
                result_future.wait();

                if (result_future.get()) {
                    auto result = result_future.get();
                    if (result->complete)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
                    }
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /approach_shelf");
                }
            }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproach>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}