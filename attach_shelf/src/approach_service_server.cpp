#include "rclcpp/rclcpp.hpp"
#include "approach_shelf_msg/srv/go_to_loading.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <memory>
#include <cmath>

using GoToLoading = approach_shelf_msg::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
    ServerNode() : Node("approach_shelf_server") {
        srv_ = create_service<GoToLoading>("approach_shelf", std::bind(&ServerNode::approach_callback, this, _1, _2));
        laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ServerNode::laser_scan_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ServerNode::odomCallback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        transform_listener_ =
            std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        kp_distance_ = 0.0;
        kp_yaw_ = 1.4;
        distance_threshold_ = 0.05;
    }

private:
    rclcpp::Service<GoToLoading>::SharedPtr srv_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<float> leg_locations_;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double kp_distance_;
    double kp_yaw_;
    double distance_threshold_;

    void approach_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response) {
        if (request->attach_to_shelf == true) {
            leg_locations_.clear();
            if (detect_shelf_legs()) {
                RCLCPP_INFO(get_logger(), "Leg locations:");
                for (size_t i = 0; i < leg_locations_.size(); i += 2) {
                    RCLCPP_INFO(get_logger(), "Leg %d: x = %.2f, y = %.2f", (i / 2) + 1, leg_locations_[i], leg_locations_[i + 1]);
                }

                float center_x = (leg_locations_[0] + leg_locations_[2]) / 2.0;
                float center_y = (leg_locations_[1] + leg_locations_[3]) / 2.0;

                RCLCPP_INFO(get_logger(), "Center location: x = %.2f, y = %.2f", center_x, center_y);

                geometry_msgs::msg::TransformStamped odom_to_robot_transform;
                try {
                    odom_to_robot_transform = tf_buffer_->lookupTransform(
                        "robot_odom", "robot_base_link", tf2::TimePointZero);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(get_logger(), "Could not get transform from robot_odom to robot_base_link: %s", ex.what());
                    return;
                }

                tf2::Vector3 translation(
                    odom_to_robot_transform.transform.translation.x,
                    odom_to_robot_transform.transform.translation.y,
                    odom_to_robot_transform.transform.translation.z
                );
                tf2::Quaternion rotation(
                    odom_to_robot_transform.transform.rotation.x,
                    odom_to_robot_transform.transform.rotation.y,
                    odom_to_robot_transform.transform.rotation.z,
                    odom_to_robot_transform.transform.rotation.w
                );

                tf2::Transform odom_to_robot_tf(rotation, translation);

                tf2::Vector3 center_robot_frame(center_x, center_y, 0.0);

                tf2::Vector3 center_odom_frame = odom_to_robot_tf * center_robot_frame;

                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = now();
                transform.header.frame_id = "robot_odom";
                transform.child_frame_id = "cart_frame";
                transform.transform.translation.x = center_odom_frame.x();
                transform.transform.translation.y = center_odom_frame.y();
                transform.transform.translation.z = 0.0;
                transform.transform.rotation.x = 0.0;
                transform.transform.rotation.y = 0.0;
                transform.transform.rotation.z = 0.0;
                transform.transform.rotation.w = 1.0;

                tf_broadcaster_->sendTransform(transform);
                
                response->complete = true;
                RCLCPP_INFO(get_logger(), "Shelf legs detected. Approaching complete.");
            } else {
                response->complete = false;
                RCLCPP_INFO(get_logger(), "Shelf legs not detected. Approaching incomplete.");
            }
        } else if (request->attach_to_shelf == false) {
            response->complete = false;
            RCLCPP_INFO(get_logger(), "Attach to shelf request set to false. Approaching incomplete.");
        }
    }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "robot_base_link", "cart_frame", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        double dist = sqrt(pow(transform.transform.translation.x, 2) +
                        pow(transform.transform.translation.y, 2));
        double error_yaw = atan2(transform.transform.translation.y,
                                transform.transform.translation.x);

        double angular_velocity = kp_yaw_ * error_yaw;
        double linear_velocity = 0.0;

        RCLCPP_INFO(get_logger(), "Distance: %f", dist);

        if (dist > distance_threshold_) {
            kp_distance_ = 1.06;
            linear_velocity = std::max(kp_distance_ * dist, 0.7);
        } else {
            kp_distance_ = 0.0;
            linear_velocity = kp_distance_ * dist;
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_velocity;
        twist.angular.z = angular_velocity;
        cmd_vel_pub_->publish(twist);
    }

    bool detect_shelf_legs() {
        if (!last_scan_) {
            RCLCPP_WARN(get_logger(), "No laser scan data available yet.");
            return false;
        }

        float intensity_threshold = 7000.0;
        int num_shelf_legs = 0;
        bool in_group = false;
        float group_start_angle = 0.0;
        float group_end_angle = 0.0;
        float group_start_range = 0.0;
        float group_end_range = 0.0;

        leg_locations_.clear();

        for (size_t i = 0; i < last_scan_->intensities.size(); ++i) {
            const auto& intensity = last_scan_->intensities[i];
            const auto& range = last_scan_->ranges[i];
            const auto angle = last_scan_->angle_min + i * last_scan_->angle_increment;

            if (intensity > intensity_threshold) {
            if (!in_group) {
                num_shelf_legs++;
                in_group = true;
                group_start_angle = angle;
                group_start_range = range;
            }
            group_end_angle = angle;
            group_end_range = range;
            } else {
            if (in_group) {
                float group_center_angle = (group_start_angle + group_end_angle) / 2.0;
                float group_center_range = (group_start_range + group_end_range) / 2.0;
                float leg_x = group_center_range * std::cos(group_center_angle);
                float leg_y = group_center_range * std::sin(group_center_angle);
                leg_locations_.push_back(leg_x);
                leg_locations_.push_back(leg_y);
            }
            in_group = false;
            }
        }

        RCLCPP_INFO(get_logger(), "Number of shelf legs detected: %d", num_shelf_legs);
        return num_shelf_legs == 2;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServerNode>());
    rclcpp::shutdown();
    return 0;
}