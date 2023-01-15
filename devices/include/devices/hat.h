#pragma once

#include "devices/gpio/pwm.h"

#include <memory>

namespace devices {

class PWM;
class Motor;
class MotorPins;
class MotorConfig;

class HAT {
public:
  // HAT(
  //   PWM& pwm,
  //   MotorConfig& left_motor_config,
  //   MotorConfig& right_motor_config);
  
  HAT(
    const PWM& pwm,
    const MotorConfig& left_motor_config,
    const MotorConfig& right_motor_config);

  void RunRightMotor(double velocity);
  void RunLeftMotor(double velocity);

private:
  int right_motor_in1_pin_ = 10;
  int right_motor_in2_pin_ = 9;
  int right_motor_pwm_pin_ = 8;
  
  std::shared_ptr<Motor> left_motor_ = nullptr;
  std::shared_ptr<Motor> right_motor_= nullptr;

  std::shared_ptr<Motor> CreateMotor(const PWM& pwm, const MotorConfig& motor_config);
};

} // namespace devices
