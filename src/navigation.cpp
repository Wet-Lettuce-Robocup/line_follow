#include "line_follow/navigation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <functional>

using std::placeholders::_1;

NavigationNode::NavigationNode()
: rclcpp_lifecycle::LifecycleNode("navigation") {}

CallbackReturn NavigationNode::on_configure(const rclcpp_lifecycle::State &)
{
  this->errorPub = this->create_publisher<std_msgs::msg::Float32>("line_error", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn NavigationNode::on_activate(const rclcpp_lifecycle::State &)
{
  this->imageSub = this->create_subscription<sensor_msgs::msg::Image>("camera_front/image_raw", 10,
    std::bind(&NavigationNode::imageCallback, this, _1));

  return CallbackReturn::SUCCESS;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
