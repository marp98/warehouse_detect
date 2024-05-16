#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace my_components
{
class PreApproach : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC explicit PreApproach(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  bool is_rotating_ = false;
  double obstacle_distance_ = 0.3;  

  void moveRobot();
  void stopRobot();
};
}  // namespace my_components

#endif  // COMPOSITION__PREAPPROACH_COMPONENT_HPP_