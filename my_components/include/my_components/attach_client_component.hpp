#ifndef COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_

#include "approach_shelf_msg/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  rclcpp::Client<approach_shelf_msg::srv::GoToLoading>::SharedPtr client_;
  void send_request();
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_