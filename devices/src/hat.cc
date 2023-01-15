#include <devices/hat.h>
#include <devices/motor.h>
#include <devices/gpio/pwm.h>

#include <iostream>
#include <string>

namespace devices {

HAT::HAT(const PWM& pwm,
  const MotorConfig& left_motor_config,
  const MotorConfig& right_motor_config) {
  left_motor_ = CreateMotor(std::move(pwm), std::move(left_motor_config)); 
  right_motor_ = CreateMotor(std::move(pwm), std::move(right_motor_config));
}

std::shared_ptr<Motor> HAT::CreateMotor(const PWM& pwm, const MotorConfig& motor_config) {
  return std::move(std::make_shared<Motor>(std::move(pwm), std::move(motor_config)));
}

void HAT::RunRightMotor(double velocity) {
  right_motor_->Run(velocity);
}

void HAT::RunLeftMotor(double velocity) {
  left_motor_->Run(velocity);
}

} // namespace devices
