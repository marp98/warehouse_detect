#include "my_components/attach_server_component.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using namespace std::this_thread; 
using namespace std::chrono; 

using namespace std::chrono_literals;
using approach_shelf_msg::srv::GoToLoading;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server", options) {
  // Initialize member variables
  kp_distance_ = 0.0;
  kp_yaw_ = 1.4;
  distance_threshold_ = 0.1;
  approach_completed_ = false;

  // Create service
  srv_ = create_service<GoToLoading>(
      "approach_shelf",
      std::bind(&AttachServer::approach_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Create subscriptions and publishers
  laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&AttachServer::laser_scan_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
  elevator_up_pub_ = create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);

  // Initialize transform broadcaster, listener, and buffer
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void AttachServer::approach_callback(const std::shared_ptr<GoToLoading::Request> request,
                    const std::shared_ptr<GoToLoading::Response> response) {
    if (request->attach_to_shelf == true) {
      leg_locations_.clear();
      if (detect_shelf_legs()) {
        RCLCPP_INFO(get_logger(), "Leg locations:");
        for (size_t i = 0; i < leg_locations_.size(); i += 2) {
          RCLCPP_INFO(get_logger(), "Leg %d: x = %.2f, y = %.2f", (i / 2) + 1,
                      leg_locations_[i], leg_locations_[i + 1]);
        }

        float center_x = (leg_locations_[0] + leg_locations_[2]) / 2.0;
        float center_y = (leg_locations_[1] + leg_locations_[3]) / 2.0;

        RCLCPP_INFO(get_logger(), "Center location: x = %.2f, y = %.2f",
                    center_x, center_y);

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

        move_robot();

        std_msgs::msg::Empty empty_msg;
        elevator_up_pub_->publish(empty_msg);

        response->complete = true;
        RCLCPP_INFO(get_logger(), "Shelf legs detected. Approaching complete.");
      } else {
        response->complete = false;
        RCLCPP_INFO(get_logger(),
                    "Shelf legs not detected. Approaching incomplete.");
      }
    } else if (request->attach_to_shelf == false) {
      response->complete = false;
      RCLCPP_INFO(
          get_logger(),
          "Attach to shelf request set to false. Approaching incomplete.");
    }
}

void AttachServer::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
}

void AttachServer::move_robot() {
    RCLCPP_INFO(get_logger(), "Starting final approach move");
    sleep_for(nanoseconds(10));
    sleep_until(system_clock::now() + seconds(1));

    while (!approach_completed_) {
      geometry_msgs::msg::TransformStamped transform_odom_to_robot;
      try {
        transform_odom_to_robot = tf_buffer_->lookupTransform(
            "robot_odom", "robot_base_link", tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Could not get transform: %s", ex.what());
        return;
      }

      double dist_odom_to_robot =
          sqrt(pow(transform_odom_to_robot.transform.translation.x, 2) +
               pow(transform_odom_to_robot.transform.translation.y, 2));
      double error_yaw_odom_to_robot =
          atan2(transform_odom_to_robot.transform.translation.y,
                transform_odom_to_robot.transform.translation.x);
      RCLCPP_INFO(get_logger(), "Distance odom to robot: %f",
                  dist_odom_to_robot);

      geometry_msgs::msg::TransformStamped transform_odom_to_cart;
      try {
        transform_odom_to_cart = tf_buffer_->lookupTransform(
            "robot_odom", "cart_frame", tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Could not get transform: %s", ex.what());
        return;
      }

      double dist_odom_to_cart =
          sqrt(pow(transform_odom_to_cart.transform.translation.x, 2) +
               pow(transform_odom_to_cart.transform.translation.y, 2));
      double error_yaw_odom_to_cart =
          atan2(transform_odom_to_cart.transform.translation.y,
                transform_odom_to_cart.transform.translation.x);
      RCLCPP_INFO(get_logger(), "Distance odom to cart: %f", dist_odom_to_cart);

      double angular_velocity = kp_yaw_ * error_yaw_odom_to_robot;
      double linear_velocity = 0.0;

      double dist_robot_to_cart =
          sqrt(pow(dist_odom_to_robot, 2) + pow(dist_odom_to_cart, 2) -
               2 * dist_odom_to_robot * dist_odom_to_cart *
                   cos(error_yaw_odom_to_cart - error_yaw_odom_to_robot));
      RCLCPP_INFO(get_logger(), "Distance: %f", dist_robot_to_cart);

      if (dist_robot_to_cart > distance_threshold_) {
        kp_distance_ = 1.06;
        linear_velocity = kp_distance_ * dist_robot_to_cart;
      } else {
        kp_distance_ = 0.0;
        linear_velocity = 0.0;

        geometry_msgs::msg::Twist extra_twist;
        extra_twist.linear.x = 0.3;  
        extra_twist.angular.z = 0.0;

        double extra_distance = 0.9;  
        double moved_distance = 0.0;
        rclcpp::Rate extra_rate(10);  

        auto start_time = now();
        while (moved_distance < extra_distance) {
            cmd_vel_pub_->publish(extra_twist);
            extra_rate.sleep();
            auto current_time = now();
            auto duration = current_time - start_time;
            moved_distance = extra_twist.linear.x * duration.seconds();
        }

        extra_twist.linear.x = 0.0;
        cmd_vel_pub_->publish(extra_twist);

        approach_completed_ = true;
      }

      geometry_msgs::msg::Twist twist;
      twist.linear.x = linear_velocity;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);

      if (approach_completed_) {
        RCLCPP_INFO(get_logger(), "Final approach completed");
        break;
      }
    }
}

bool AttachServer::detect_shelf_legs() {
    if (!last_scan_) {
      RCLCPP_WARN(get_logger(), "No laser scan data available yet.");
      return false;
    }

    float intensity_threshold = 7000.0;
    float min_leg_distance = 0.1;
    int num_shelf_legs = 0;
    bool in_group = false;
    float group_start_angle = 0.0;
    float group_end_angle = 0.0;
    float group_start_range = 0.0;
    float group_end_range = 0.0;

    leg_locations_.clear();

    for (size_t i = 0; i < last_scan_->intensities.size(); ++i) {
      const auto &intensity = last_scan_->intensities[i];
      const auto &range = last_scan_->ranges[i];
      const auto angle =
          last_scan_->angle_min + i * last_scan_->angle_increment;

      if (intensity > intensity_threshold && range > min_leg_distance) {
        if (!in_group) {
          in_group = true;
          group_start_angle = angle;
          group_start_range = range;
        }
        group_end_angle = angle;
        group_end_range = range;
      } else {
        if (in_group) {
          float leg_x = 0.0;
          float leg_y = 0.0;
          int num_points = 0;

          for (float a = group_start_angle; a <= group_end_angle;
               a += last_scan_->angle_increment) {
            size_t index =
                (a - last_scan_->angle_min) / last_scan_->angle_increment;
            float r = last_scan_->ranges[index];
            if (r > min_leg_distance) {
              leg_x += r * std::cos(a);
              leg_y += r * std::sin(a);
              num_points++;
            }
          }

          if (num_points > 0) {
            leg_x /= num_points;
            leg_y /= num_points;
            leg_locations_.push_back(leg_x);
            leg_locations_.push_back(leg_y);
            num_shelf_legs++;
          }
          in_group = false;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Number of shelf legs detected: %d",
                num_shelf_legs);
    return num_shelf_legs == 2;
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)