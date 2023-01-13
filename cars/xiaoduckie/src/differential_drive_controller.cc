#include <xiaoduckie/differential_drive_controller.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace xiaocars {

using std::placeholders::_1;
using namespace std::chrono_literals;


DifferentialDriveController::DifferentialDriveController(
  double wheel_base,
  double wheel_radius)
  : Node("DifferentialDriveController"),
  wheel_base_{wheel_base},
  wheel_radius_{wheel_radius} {
    std::string channel_name = std::string("cmd_vel");
    RCLCPP_INFO(
      this->get_logger(),
      ": DifferentialDriveController: Creating publisher for topic '%s' ",
      channel_name.c_str());
    publisher_ = this->create_publisher<std_msgs::msg::String>(channel_name, 10);
  std::cout << ">>>>>>>>>>>>>> creating wall timer" << std::endl;
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DifferentialDriveController::timer_callback, this));
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
