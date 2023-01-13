#include <map>
#include <utility>
#include <memory>
#include <JetsonGPIO.h>

#include <devices/motor.h>
#include <devices/gpio/pwm.h>
#include <devices/gpio/constants.h>

namespace devices {

// 
Motor::Motor(
  std::string const& name,
  PWM & pwm,
  int in1_pin,
  int in2_pin,
  int pwm_pin,
  DirectionControl const control_by,
  int k)
  : name_(name),
  K_(k),
  pwm_(pwm),
  in1_pin_(in1_pin),
  in2_pin_(in2_pin),
  pwm_pin_(pwm_pin),
  control_by_(control_by),
  direction_(MotorDirection::FORWARD)
{
  
  if (control_by_ == DirectionControl::GPIO) {
    SetupByGPIO();
  }
}

void Motor::SetupByGPIO() {
  std::cout << "GPIO INIT START" << in1_pin_ << " -- " << in2_pin_ << std::endl;
  GPIO::setmode(GPIO::BOARD);
  std::cout << "GPIO INIT END" << std::endl;
  GPIO::setup(in1_pin_, GPIO::OUT);
  GPIO::setup(in2_pin_, GPIO::OUT);
  
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
  if (fabs(velocity) > constants::speed_tolerance) {
    pwm = static_cast<int>(floor(fabs(velocity))
        * (constants::max_pwm_value - constants::min_pwm_value)
        + constants::min_pwm_value);
  }
  else if (fabs(velocity) < constants::speed_tolerance) {
    return pwm;
  }
  return std::min(pwm, constants::max_pwm_value);
}

void Motor::Run(double velocity) {

  int pwm_value = GetPWMValueFromVelocity(velocity);
  direction_ = GetDirectionFromVelocity(velocity, pwm_value);

  pwm_value = std::max(0.0, std::min(velocity, 255.0));

  switch (control_by_) {
    case DirectionControl::GPIO:
      SetDirectionByGPIO();
      break;
    case DirectionControl::PWM:
      SetDirectionByPWM();
      break;
  }
  pwm_.setPWM(pwm_pin_, 0, pwm_value * K_);
}

void Motor::SetDirectionByGPIO() {
  switch (direction_) {
    case MotorDirection::RELEASE:
      GPIO::output(in1_pin_, constants::HIGH);
      GPIO::output(in2_pin_, constants::HIGH);
      break;
    case MotorDirection::FORWARD:
      GPIO::output(in1_pin_, constants::HIGH);
      GPIO::output(in2_pin_, constants::LOW);
      break;
    case MotorDirection::BACKWARD:
      GPIO::output(in1_pin_, constants::LOW);
      GPIO::output(in2_pin_, constants::HIGH);
      break;
  }
}

void Motor::SetDirectionByPWM() {
  switch (direction_) {
    case MotorDirection::RELEASE:
      pwm_.setPWM(in1_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      pwm_.setPWM(in2_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      break;
    case MotorDirection::FORWARD:
      pwm_.setPWM(in1_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      pwm_.setPWM(in2_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      break;
    case MotorDirection::BACKWARD:
      pwm_.setPWM(in1_pin_, constants::PWM_LOW, constants::PWM_HIGH);
      pwm_.setPWM(in2_pin_, constants::PWM_HIGH, constants::PWM_LOW);
      break;
  }
}

} // namespace devices
