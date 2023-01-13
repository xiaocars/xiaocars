#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

namespace control {

class DifferentialDriveController {
public:
  DifferentialDriveController(std::shared_ptr<rclcpp::Node> & nh, double wheel_base, double wheel_radius);
private:
  std::shared_ptr<rclcpp::Node> & nh_;
  double wheel_base_;
  double wheel_radius_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  // void CommandVelocityListener(geometry_msgs::msg::Twist cmd_vel);
};
} // namespace control
  