#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

#include <devices/hat.h>

namespace xiaoduckie {

class WheelsDriver {
public:
  WheelsDriver(
    std::shared_ptr<rclcpp::Node>& nh,
    devices::HAT& hat);
private:
  std::shared_ptr<rclcpp::Node> & nh_;
  devices::HAT& hat_;
  rclcpp::Subscription<xiaocar_msgs::msg::WheelsCmdStamped>::SharedPtr subscription_;

  void WheelCommandListener(xiaocar_msgs::msg::WheelsCmdStamped wheels_cmd);
};

} // namespace xiaoduckie