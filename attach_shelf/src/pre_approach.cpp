#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PreApproachNode : public rclcpp::Node
{
public:
  PreApproachNode() : Node("pre_approach")
  {
    this->declare_parameter("obstacle", 1.0);
    this->declare_parameter("degrees", 90.0);

    obstacle_distance_ = this->get_parameter("obstacle").as_double();
    rotation_degrees_ = this->get_parameter("degrees").as_double();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PreApproachNode::scanCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  double obstacle_distance_;
  double rotation_degrees_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double front_range = msg->ranges[msg->ranges.size() / 2];

    geometry_msgs::msg::Twist twist;

    if (front_range > obstacle_distance_)
    {
      twist.linear.x = 0.2;  
      twist.angular.z = 0.0;
    }
    else
    {
      twist.linear.x = 0.0;
      twist.angular.z = (rotation_degrees_ * M_PI) / 180.0;  
    }

    cmd_vel_pub_->publish(twist);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}