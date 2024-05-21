#include "my_components/attach_client_component.hpp"
#include <cinttypes>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "approach_shelf_msg/srv/go_to_loading.hpp"

using namespace std::chrono_literals;
using GoToLoading = approach_shelf_msg::srv::GoToLoading;
using ServiceResponseFuture = rclcpp::Client<approach_shelf_msg::srv::GoToLoading>::SharedFuture;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  RCLCPP_INFO(get_logger(), "AttachClient constructor called");
  client_ = create_client<GoToLoading>("approach_shelf");
  
  RCLCPP_INFO(get_logger(), "Waiting for service...");
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }
  
  RCLCPP_INFO(get_logger(), "Service available, sending request...");
  send_request();
}

void AttachClient::send_request() {
  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true;
  
  RCLCPP_INFO(get_logger(), "Sending request with attach_to_shelf = true");
  
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto response = future.get();
      RCLCPP_INFO(get_logger(), "Response received: %s", response->complete ? "Approach completed" : "Approach incomplete");
    } else {
      RCLCPP_INFO(get_logger(), "Service In-Progress...");
    }
  };
  
  auto future_result = client_->async_send_request(request, response_received_callback);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)