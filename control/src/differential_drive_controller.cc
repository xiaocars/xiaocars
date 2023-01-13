#include <control/differential_drive_controller.h>

namespace control {

using std::placeholders::_1;

DifferentialDriveController::DifferentialDriveController(
  std::shared_ptr<rclcpp::Node> & nh,
  double wheel_base,
  double wheel_radius)
  : nh_{nh},
  wheel_base_{wheel_base},
  wheel_radius_{wheel_radius} {
    std::string channel_name = std::string("/xiaoduckie/cmd_wheels_vel");
    RCLCPP_INFO(
      nh_->get_logger(),
      ": DifferentialDriveController: Creating publisher for topic '%s' ",
      channel_name.c_str());
    publisher_ = nh_->create_publisher<std_msgs::msg::Int32>(channel_name, 10); 
  }
  // subscription_{
  //   nh_->create_subscription<geometry_msgs::msg::Twist>(
  //     "/xiaoduckie/cmd_vel",
  //     10,
  //     std::bind(&DifferentialDriveController::CommandVelocityListener, this, _1))} 

// void DifferentialDriveController::CommandVelocityListener(std_msgs::msg::Int32 cmd_vel) {
  // auto message = std_msgs::msg::Int32();
  // message.vel_left = ((2.0 *cmd_vel.linear.x) + (cmd_vel.angular.z * wheel_base_)) / (2 * wheel_radius_);
  // message.vel_right = ((2.0 *cmd_vel.linear.x) - (cmd_vel.angular.z * wheel_base_)) / (2 * wheel_radius_);

  // RCLCPP_DEBUG(
  //   nh_->get_logger(), 
  //   ":DifferentialDriveController: publishing velocity left '%f' and velocity right '%f'",
  //   message.vel_left,
  //   message.vel_right);
  
//   publisher_->publish(message);
// }

} // namespace control
