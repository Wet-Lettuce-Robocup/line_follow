#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PIDLoop : public rclcpp_lifecycle::LifecycleNode
{
public:
  PIDLoop();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr errorSub;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> twistPub;

  double kp;
  double ki;
  double kd;

  double defaultSpeed;

  double integral;
  double lastError;
  rclcpp::Time lastTime;

  void errorCallback(std_msgs::msg::Float64::SharedPtr error);
};
