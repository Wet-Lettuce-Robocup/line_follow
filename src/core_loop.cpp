#include "line_follow/core_loop.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <memory>

CoreLoop::CoreLoop() : rclcpp_lifecycle::LifecycleNode("core_loop")
{
  RCLCPP_INFO(this->get_logger(), "test");
}

CallbackReturn CoreLoop::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "configured");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreLoop::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "activated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreLoop::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "deactivated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreLoop::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "cleanup");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreLoop::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "shutdown");

  return CallbackReturn::SUCCESS;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoreLoop>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
