#pragma once

#include <map>
#include <utility>
#include <memory>

#include <JetsonGPIO.h>
#include <devices/gpio/pwm.h>

namespace devices {

// 
enum class MotorDirection {
  RELEASE = 0,
  FORWARD = 1,
  BACKWARD = -1
};

enum class DirectionControl {
  PWM = 0,
  GPIO = 1
};

class MotorPins {
public:
  MotorPins(int in1_pin, int in2_pin, int pwm_pin);
private:
  int in1_pin_;
  int in2_pin_;
  int pwm_pin_;
};

class Motor {
private:
  std::string const& name_;
  int const K_;
  PWM & pwm_;
  int const in1_pin_;
  int const in2_pin_;
  int const pwm_pin_;
  DirectionControl const control_by_;
  MotorDirection direction_;

  void SetupByGPIO();
  void SetDirectionByGPIO();
  void SetDirectionByPWM();
  int GetPWMValueFromVelocity(double velocity);
  MotorDirection GetDirectionFromVelocity(double velocity, int pwm_value);
public:

  Motor(
    std::string const& name,
    PWM & pwm,
    int in1_pin,
    int in2_pin,
    int pwm_pin,
    DirectionControl const control_by,
    int const k = 16);

  void Run(double velocity = 0);
};

} // namespace devices
