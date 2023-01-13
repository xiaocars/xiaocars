#include <xiaoduckie/wheels_driver.h>

namespace xiaoduckie {

using std::placeholders::_1;

WheelsDriver::WheelsDriver(
  std::shared_ptr<rclcpp::Node>& nh,
  devices::HAT& hat)
  : nh_{nh}, hat_{hat}, 
  subscription_{
    nh->create_subscription<xiaocar_msgs::msg::WheelsCmdStamped>(
      "/xiaoduckie/cmd_wheels_vel",
      10,
      std::bind(&WheelsDriver::WheelCommandListener, this, _1) )} {}

void WheelsDriver::WheelCommandListener(xiaocar_msgs::msg::WheelsCmdStamped wheel_cmd) {

  RCLCPP_DEBUG(
    nh_->get_logger(),
    "WheelsDriver: received velocity left '%f' and velocity right '%f",
    wheel_cmd.vel_left,
    wheel_cmd.vel_right);

  hat_.RunLeftMotor(wheel_cmd.vel_left);
  hat_.RunRightMotor(wheel_cmd.vel_right);
}

}
