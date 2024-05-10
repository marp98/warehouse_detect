#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class PreApproachNode : public rclcpp::Node
{
public:
    PreApproachNode() : Node("pre_approach_node"), obstacle_distance_(0.3), rotation_degrees_(-90), initial_yaw_(0.0), start_pose_set_(false)
    {
        this->declare_parameter("obstacle", obstacle_distance_);
        this->declare_parameter("degrees", rotation_degrees_);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproachNode::laserCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PreApproachNode::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    double obstacle_distance_;
    int rotation_degrees_;
    double initial_yaw_;
    bool is_rotating_ = false;
    bool start_pose_set_;
    double start_yaw_;
    double target_yaw_;
    geometry_msgs::msg::Pose start_pose_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!is_rotating_)
        {
            if (msg->ranges[msg->ranges.size() / 2] <= obstacle_distance_)
            {
                stopRobot();
                is_rotating_ = true;
                RCLCPP_INFO(this->get_logger(), "Obstacle detected");
            }
            else
            {
                moveRobot();
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (is_rotating_) {
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

            double yaw_tolerance = 0.001;  

            if (std::abs(yaw - target_yaw_) < yaw_tolerance) {
                stopRobot();
                is_rotating_ = false;
                RCLCPP_INFO(this->get_logger(), "Rotation completed. Terminating the program");
                RCLCPP_INFO(this->get_logger(), "Final Yaw: %.2f", yaw);
                rclcpp::shutdown();
            } else {
                rotateRobot();
            }
        }
    }

    void moveRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.4;
        cmd_vel_pub_->publish(cmd_vel);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Stopping the robot:");
    }

    void setUpRotate() {
        double target_yaw = 0 + rotation_degrees_ * M_PI / 180.0;
        target_yaw_ = target_yaw;

        RCLCPP_INFO(this->get_logger(), "Target Yaw: %.2f", target_yaw_);
    }

    void rotateRobot()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.2 * std::copysign(1.0, rotation_degrees_);
        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproachNode>();
    RCLCPP_INFO(node->get_logger(), "Starting move");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}