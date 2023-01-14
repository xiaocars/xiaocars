#include <control/differential_drive_controller.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

namespace control {

using std::placeholders::_1;

DifferentialDriveController::DifferentialDriveController(
  std::string& cmd_vel_topic_name,
  std::string& wheels_vel_topic_name,
  double wheel_base,
  double wheel_radius)
  : Node("DifferentialDriveController"),
  wheel_base_{wheel_base},
  wheel_radius_{wheel_radius} {

    RCLCPP_INFO(
      this->get_logger(),
      ": DifferentialDriveController: Creating publisher for topic '%s' ",
      wheels_vel_topic_name.c_str());

    publisher_ = this->create_publisher<xiaocar_msgs::msg::WheelsCmdStamped>(
      wheels_vel_topic_name,
      10);

    RCLCPP_INFO(
      this->get_logger(),
      ": DifferentialDriveController: Creating Subscription for topic '%s' ",
      cmd_vel_topic_name.c_str());

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_name,
      10,
      std::bind(&DifferentialDriveController::CommandVelocityListener, this, _1));

  }
  

void DifferentialDriveController::CommandVelocityListener(geometry_msgs::msg::Twist cmd_vel) {
  auto message = xiaocar_msgs::msg::WheelsCmdStamped();
  message.vel_left = ((2.0 *cmd_vel.linear.x) + (cmd_vel.angular.z * wheel_base_)) / (2 * wheel_radius_);
  message.vel_right = ((2.0 *cmd_vel.linear.x) - (cmd_vel.angular.z * wheel_base_)) / (2 * wheel_radius_);

  RCLCPP_DEBUG(
    this->get_logger(), 
    ":DifferentialDriveController: publishing velocity left '%f' and velocity right '%f'",
    message.vel_left,
    message.vel_right);
  
  publisher_->publish(message);
}

} // namespace control
