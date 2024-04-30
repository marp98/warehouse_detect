#include "rclcpp/rclcpp.hpp"
#include "approach_shelf_msg/srv/go_to_loading.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

using GoToLoading = approach_shelf_msg::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
    ServerNode() : Node("approach_shelf_server") {
        srv_ = create_service<GoToLoading>("approach_shelf", std::bind(&ServerNode::approach_callback, this, _1, _2));
        laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ServerNode::laser_scan_callback, this, _1));
    }

private:
    rclcpp::Service<GoToLoading>::SharedPtr srv_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    void approach_callback(const std::shared_ptr<GoToLoading::Request> request,
                           const std::shared_ptr<GoToLoading::Response> response) {
        if (request->attach_to_shelf == true) {
            if (detect_shelf_legs()) {
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

    bool detect_shelf_legs() {
        if (!last_scan_) {
            RCLCPP_WARN(get_logger(), "No laser scan data available yet.");
            return false;
        }
        
        float intensity_threshold = 7000.0;  
        int num_shelf_legs = 0;
        bool in_group = false;

        for (const auto& intensity : last_scan_->intensities) {
            RCLCPP_INFO(get_logger(), "Intensity: %.2f", intensity);

            if (intensity > intensity_threshold) {
                if (!in_group) {
                    num_shelf_legs++;
                    in_group = true;
                }
            } else {
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