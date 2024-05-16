#include "my_components/preapproach_component.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: Node("preapproach_node", options)
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

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, laser_callback);
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
}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)