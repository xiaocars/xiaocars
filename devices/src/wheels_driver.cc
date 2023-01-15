#include "devices/wheels_driver.h"
#include "devices/hat.h"

#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <memory>

namespace devices {

using std::placeholders::_1;

DifferentialDriveWheelsDriver::DifferentialDriveWheelsDriver(
  rclcpp::NodeOptions& options,
  devices::HAT& hat,
  std::string& topic_name)
  : Node("DifferentialDriveWheelsDriver", options)
  , hat_{hat}
  , topic_name_{topic_name} {

    CreateSubscription(topic_name);
  }

void DifferentialDriveWheelsDriver::CreateSubscription(std::string& topic_name) {

  RCLCPP_DEBUG(
    this->get_logger(),
    "Creating a subscription to topic '%s'",
    topic_name.c_str());

  subscription_ = this->create_subscription<xiaocar_msgs::msg::WheelsCmdStamped>(
    topic_name,
    10,
    std::bind(&DifferentialDriveWheelsDriver::WheelCommandListener, this, _1));
}

void DifferentialDriveWheelsDriver::WheelCommandListener(
  xiaocar_msgs::msg::WheelsCmdStamped wheel_cmd) {

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received velocity left '%f' and velocity right '%f",
    wheel_cmd.vel_left,
    wheel_cmd.vel_right);

  std::cout << "Received velocity left " 
    << wheel_cmd.vel_left << " and velocity right " 
    << wheel_cmd.vel_right << std::endl;

  hat_.RunLeftMotor(wheel_cmd.vel_left);
  hat_.RunRightMotor(wheel_cmd.vel_right);
}

} // namespace devices
