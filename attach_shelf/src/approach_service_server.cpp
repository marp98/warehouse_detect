#include "rclcpp/rclcpp.hpp"
#include "approach_shelf_msg/srv/go_to_loading.hpp"

#include <memory>

using GoToLoading = approach_shelf_msg::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("approach_shelf_server")
  {

    srv_ = create_service<GoToLoading>("approach_shelf", std::bind(&ServerNode::approach_callback, this, _1, _2));

  }

private:
  rclcpp::Service<GoToLoading>::SharedPtr srv_;

  void approach_callback(
      const std::shared_ptr<GoToLoading::Request> request,
      const std::shared_ptr<GoToLoading::Response>
          response) 
    {    
        if (request->attach_to_shelf == true)
        {       
            response->complete = true;
        }
        else if (request->attach_to_shelf == false)
        {
            response->complete = false;
        }                
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}