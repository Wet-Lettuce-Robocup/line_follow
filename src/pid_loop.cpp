#include "line_follow/pid_loop.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <cstdint>

using std::placeholders::_1;

PIDLoop::PIDLoop()
: rclcpp_lifecycle::LifecycleNode("pid_loop")
{
  this->declare_parameter<double>("kp", 0.1);
  this->declare_parameter<double>("ki", 0.0);
  this->declare_parameter<double>("kd", 0.0);
  this->declare_parameter<double>("default_speed", 0.05);

  this->kp = this->get_parameter("kp").as_double();
  this->ki = this->get_parameter("ki").as_double();
  this->kd = this->get_parameter("kd").as_double();
  this->defaultSpeed = this->get_parameter("default_speed").as_double();
}

CallbackReturn PIDLoop::on_configure(const rclcpp_lifecycle::State &)
{
  this->twistPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  this->errorSub = this->create_subscription<std_msgs::msg::Float64>("line_error", 10,
    std::bind(&PIDLoop::errorCallback, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDLoop::on_activate(const rclcpp_lifecycle::State &)
{
  this->twistPub->on_activate();

  this->integral = 0;
  this->lastError = 0;
  this->lastTime = this->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDLoop::on_deactivate(const rclcpp_lifecycle::State &)
{
  this->twistPub->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDLoop::on_cleanup(const rclcpp_lifecycle::State &)
{
  this->twistPub.reset();
  this->errorSub.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDLoop::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void PIDLoop::errorCallback(std_msgs::msg::Float64::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  rclcpp::Time currentTime = this->get_clock()->now();
  int64_t dt = (currentTime - this->lastTime).nanoseconds();
  if (dt <= 0) {
    return;
  }

  double error = msg->data;
  this->integral += error * dt;
  double derivative = (error - this->lastError) / dt;

  this->lastError = error;
  this->lastTime = currentTime;

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = this->defaultSpeed;
  twist_msg.angular.z = error * this->kp + this->integral * this->ki + derivative * this->kd;

  this->twistPub->publish(twist_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PIDLoop>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
