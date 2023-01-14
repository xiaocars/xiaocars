#pragma once

#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

namespace devices {

class HAT;

class DifferentialDriveWheelsDriver : public rclcpp::Node {
public:
  DifferentialDriveWheelsDriver(devices::HAT& hat, std::string& topic_name);
private:
  devices::HAT& hat_;
  std::string& topic_name_;
  rclcpp::Subscription<xiaocar_msgs::msg::WheelsCmdStamped>::SharedPtr subscription_ = nullptr;

  void CreateSubscription(std::string& topic_name);
  void WheelCommandListener(xiaocar_msgs::msg::WheelsCmdStamped wheels_cmd);
};

} // namespace xiaoduckie
