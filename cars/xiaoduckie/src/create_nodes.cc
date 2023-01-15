#include "xiaoduckie/create_nodes.h"

#include <utility>
#include <memory>

devices::MotorConfig CreateLeftMotorConfig() {

  devices::MotorPins pins = {10, 9, 8};
  
  devices::MotorConfig config = {
    std::string("left"),
    devices::DirectionControl::PWM,
    pins
  };

  return config;
}

devices::MotorConfig CreateRightMotorConfig() {

  devices::MotorPins pins = {33, 31, 13};
  
  devices::MotorConfig config = {
    std::string("right"),
    devices::DirectionControl::GPIO,
    pins
  };

  return config;
}


devices::PWM Build_PWM() {
  devices::PWM pwm = devices::PWM(0x60);
  return pwm;
}

devices::HAT Build_HAT() {

  devices::PWM pwm = devices::PWM(0x60);

  devices::MotorConfig left_motor_config = CreateLeftMotorConfig();

  devices::MotorConfig right_motor_config = CreateRightMotorConfig();
  
  devices::HAT hat = devices::HAT(
    pwm,
    left_motor_config,
    right_motor_config);

  return hat;
}

std::shared_ptr<devices::DifferentialDriveWheelsDriver>Build_DifferentialDriveWheelsDriver(
  rclcpp::NodeOptions& options,
  devices::HAT& hat,
  std::string& wheels_vel_topic_name) {
  
  return std::make_shared<devices::DifferentialDriveWheelsDriver>(
    options,
    hat,
    wheels_vel_topic_name);
}

std::shared_ptr<control::DifferentialDriveController> Build_DifferentialDriveController(
  rclcpp::NodeOptions& options,
  std::string& cmd_vel_topic_name,
  std::string& wheels_vel_topic_name) {

  double wheel_base = 7.62;
  double wheel_radius = 3.5;

  return std::make_shared<control::DifferentialDriveController>(
    options,
    cmd_vel_topic_name,
    wheels_vel_topic_name,
    wheel_base,
    wheel_radius
  );
}