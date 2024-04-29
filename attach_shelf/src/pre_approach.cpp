#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach"), is_rotating_(false), is_stopped_(false) {
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
  bool is_rotating_;
  bool is_stopped_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan");
        double front_range = msg->ranges[msg->ranges.size() / 2];
        RCLCPP_INFO(this->get_logger(), "Front range: %f", front_range);

        geometry_msgs::msg::Twist twist;
        if (front_range > obstacle_distance_ && !is_rotating_ && !is_stopped_) {
            twist.linear.x = 0.4;
            twist.angular.z = 0.0;
        } else if (!is_rotating_ && !is_stopped_) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            is_rotating_ = true;
            rotate();
        }

        cmd_vel_pub_->publish(twist);
    }

    void rotate() {
        double target_rotation = rotation_degrees_ * M_PI / 180.0;
        double angular_velocity = 0.2; // rad/s
        double rotation_time = (std::abs(target_rotation) / angular_velocity) + 1.4; 
        double elapsed_time = 0.0;

        RCLCPP_INFO(this->get_logger(), "Rotation time: %f seconds", rotation_time);

        rclcpp::Rate rate(10); // 10 Hz
        while (elapsed_time < rotation_time) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = std::copysign(angular_velocity, rotation_degrees_);
            cmd_vel_pub_->publish(twist);

            elapsed_time += 0.1; // Increment by 0.1 seconds (10 Hz)
            RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds", elapsed_time);
            rate.sleep();
        }

        geometry_msgs::msg::Twist stop_twist;
        stop_twist.linear.x = 0.0;
        stop_twist.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_twist);

        is_rotating_ = false;
        is_stopped_ = true;
        RCLCPP_INFO(this->get_logger(), "Rotation completed");
    }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}