#include <devices/gpio/pwm.h>
#include <devices/hat.h>
#include <devices/motor.h>
#include <devices/wheels_driver.h>

#include <sensors/wheel_encoder.h>

#include <control/differential_drive_controller.h>

devices::PWM Build_PWM();

devices::HAT Build_HAT();

std::shared_ptr<devices::DifferentialDriveWheelsDriver>Build_DifferentialDriveWheelsDriver(
  rclcpp::NodeOptions& options,
  devices::HAT& hat,
  std::string& wheels_vel_topic_name);

std::shared_ptr<control::DifferentialDriveController> Build_DifferentialDriveController(
  rclcpp::NodeOptions& options,
  std::string& cmd_vel_topic_name,
  std::string& wheels_vel_topic_name);
