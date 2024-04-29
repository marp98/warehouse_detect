#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {
    this->declare_parameter("obstacle");
    this->declare_parameter("degrees");

    obstacle_distance_ = this->get_parameter("obstacle").as_double();
    rotation_degrees_ = this->get_parameter("degrees").as_int();

    RCLCPP_INFO(this->get_logger(), "Obstacle distance: %f",
                obstacle_distance_);
    RCLCPP_INFO(this->get_logger(), "Rotation degrees: %d", rotation_degrees_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  double obstacle_distance_;
  int rotation_degrees_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double front_range = msg->ranges[msg->ranges.size() / 2];
        RCLCPP_INFO(this->get_logger(), "Front range: %f", front_range);

        geometry_msgs::msg::Twist twist;
        if (front_range > obstacle_distance_) {
            twist.linear.x = 0.4;
            twist.angular.z = 0.0;
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}