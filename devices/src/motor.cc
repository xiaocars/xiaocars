#include "devices/motor.h"

#include <devices/gpio/pwm.h>
#include <devices/constants.h>

#include <JetsonGPIO.h>

#include <cmath>
#include <iostream>

namespace devices {

Motor::Motor(
  const PWM& pwm,
  const MotorConfig& motor_config)
  : pwm_{pwm}
  , motor_config_{motor_config}
  , direction_{MotorDirection::FORWARD}
{
  if (motor_config_.control_by_ == DirectionControl::GPIO) {
    SetupByGPIO();
  }
}

Motor::Motor(
  const PWM&& pwm,
    const MotorConfig&& motor_config)
  : pwm_{std::move(pwm)}
  , motor_config_{std::move(motor_config)}
  , direction_{MotorDirection::FORWARD}
{
  if (motor_config_.control_by_ == DirectionControl::GPIO) {
    SetupByGPIO();
  }
}

void Motor::SetupByGPIO() {
  std::cout << "GPIO INIT START" << motor_config_.motor_pins_.in1_pin_ << " -- " << motor_config_.motor_pins_.in2_pin_ << std::endl;
  GPIO::setmode(GPIO::BOARD);
  std::cout << "GPIO INIT END" << std::endl;
  GPIO::setup(motor_config_.motor_pins_.in1_pin_, GPIO::OUT);
  GPIO::setup(motor_config_.motor_pins_.in2_pin_, GPIO::OUT);
  
}

MotorDirection Motor::GetDirectionFromVelocity(double velocity, int pwm_value) {
  auto direction = MotorDirection::RELEASE;
  
  if (velocity > 0 && pwm_value > 0) {
    direction =  MotorDirection::FORWARD;
  } else if (velocity < 0) {
    direction = MotorDirection::BACKWARD;
  }
  return direction;
}

int Motor::GetPWMValueFromVelocity(double velocity) {
  int pwm = 0;
  if (std::fabs(velocity) > constants::speed_tolerance) {
    pwm = static_cast<int>(std::floor(fabs(velocity)
        * (constants::max_pwm_value - constants::min_pwm_value)
        + constants::min_pwm_value));
  }
  else if (fabs(velocity) < constants::speed_tolerance) {
    return pwm;
  }
  return std::min(pwm, constants::max_pwm_value);
}

void Motor::Run(double velocity) {
  int pwm_value = GetPWMValueFromVelocity(velocity);
  direction_ = GetDirectionFromVelocity(velocity, pwm_value);

  std::cout << "The velocity passed in is " << velocity << std::endl;
  std::cout << "   pwm_value returned " << pwm_value << std::endl;

  pwm_value = std::max(0, std::min(pwm_value, 255));
  std::cout << "Then max/min returned " << pwm_value << std::endl;
  std::cout << "  The value after the k gain is " << (pwm_value * constants::MOTOR_K_GAIN) << std::endl;

  switch (motor_config_.control_by_) {
    case DirectionControl::GPIO:
      SetDirectionByGPIO();
      break;
    case DirectionControl::PWM:
      SetDirectionByPWM();
      break;
  }
  pwm_.setPWM(motor_config_.motor_pins_.pwm_pin_, 0, pwm_value * constants::MOTOR_K_GAIN);
}

void Motor::SetDirectionByGPIO() {
  switch (direction_) {
    case MotorDirection::RELEASE:
      GPIO::output(motor_config_.motor_pins_.in1_pin_, constants::HIGH);
      GPIO::output(motor_config_.motor_pins_.in2_pin_, constants::HIGH);
      break;
    case MotorDirection::FORWARD:
      GPIO::output(motor_config_.motor_pins_.in1_pin_, constants::HIGH);
      GPIO::output(motor_config_.motor_pins_.in2_pin_, constants::LOW);
      break;
    case MotorDirection::BACKWARD:
      GPIO::output(motor_config_.motor_pins_.in1_pin_, constants::LOW);
      GPIO::output(motor_config_.motor_pins_.in2_pin_, constants::HIGH);
      break;
  }
}

void Motor::SetDirectionByPWM() {
  switch (direction_) {
    case MotorDirection::RELEASE:
      pwm_.setPWM(motor_config_.motor_pins_.in1_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      pwm_.setPWM(motor_config_.motor_pins_.in2_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      break;
    case MotorDirection::FORWARD:
      pwm_.setPWM(motor_config_.motor_pins_.in1_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      pwm_.setPWM(motor_config_.motor_pins_.in2_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      break;
    case MotorDirection::BACKWARD:
      pwm_.setPWM(motor_config_.motor_pins_.in1_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      pwm_.setPWM(motor_config_.motor_pins_.in2_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      break;
  }
}

} // namespace devices
