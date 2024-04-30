#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ApproachShelfServer : public rclcpp::Node {
public:
    ApproachShelfServer() : Node("approach_shelf_server") {
        approach_shelf_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/approach_shelf",
            std::bind(&ApproachShelfServer::approachShelfCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Approach Shelf Service Server ready.");
    }

private:
    void approachShelfCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                               const std_srvs::srv::Trigger::Response::SharedPtr response) {
        (void)request;  // Unused parameter
        response->success = true;
        response->message = "Approach Shelf service called.";
        RCLCPP_INFO(this->get_logger(), "Approach Shelf service called. Returning true.");
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr approach_shelf_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachShelfServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}