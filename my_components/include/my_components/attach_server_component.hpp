#ifndef COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_

#include "approach_shelf_msg/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <memory>

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  rclcpp::Service<approach_shelf_msg::srv::GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<float> leg_locations_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  double kp_distance_;
  double kp_yaw_;
  double distance_threshold_;
  bool approach_completed_;

  void approach_callback(
    const std::shared_ptr<approach_shelf_msg::srv::GoToLoading::Request> request,
    const std::shared_ptr<approach_shelf_msg::srv::GoToLoading::Response> response);

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void move_robot();
  bool detect_shelf_legs();
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_