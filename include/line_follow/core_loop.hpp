#include <rclcpp/client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CoreLoop : public rclcpp_lifecycle::LifecycleNode
{
public:
  CoreLoop();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void transitionClient(
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client,
    uint8_t transition);
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> navigation_client;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> pid_manager_client;
};
