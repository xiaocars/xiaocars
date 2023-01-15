#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

#include <memory>

namespace control {

class DifferentialDriveController : public rclcpp::Node  {
public:
  DifferentialDriveController(
    rclcpp::NodeOptions& options,
    std::string& cmd_vel_topic_name,
    std::string& wheels_vel_topic_name,
    double wheel_base,
    double wheel_radius);
private:
  double wheel_base_;
  double wheel_radius_;
  rclcpp::Publisher<xiaocar_msgs::msg::WheelsCmdStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  void CommandVelocityListener(geometry_msgs::msg::Twist cmd_vel);
};

} // namespace control
  