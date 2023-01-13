#include <xiaoduckie/xiaoduckie.h>
#include <sensors/wheel_encoder.h>
#include <devices/hat.h>
#include <xiaoduckie/wheels_driver.h>
#include <xiaoduckie/differential_drive_controller.h>

namespace xiaocars
{
XiaoDuckie::XiaoDuckie()
: Node("XiaoDuckie"), count_(0)
{
  auto node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
  auto config = sensors::WheelEncoderConfig {
    12,
    135,
    30,
    10,
    "left",
    "/xiaoduckie/left_wheel_encoder"
  };
  // auto encoder = sensors::WheelEncoder(
  //   node_handle_,
  //   config
  // );

  // auto hat = devices::HAT(0x60, 1600.0);
  // auto wheels_driver = xiaoduckie::WheelsDriver(node_handle_, hat);
  auto controller = xiaocars::DifferentialDriveController(1.0, 1.0);
}        
} // namespace xiaocars
