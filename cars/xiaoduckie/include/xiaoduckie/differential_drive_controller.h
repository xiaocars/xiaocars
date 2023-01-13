#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace xiaocars {

class DifferentialDriveController : public rclcpp::Node {
public:
  DifferentialDriveController(double wheel_base, double wheel_radius);
private:

  double wheel_base_;
  double wheel_radius_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ ;
  int count_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_ = nullptr;

  // void CommandVelocityListener(geometry_msgs::msg::Twist cmd_vel);
  void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      std::cout << message.data << std::endl;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
};
} // namespace control
