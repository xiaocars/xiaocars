#pragma once

#include <string>

namespace devices {

class PWM;

enum class MotorDirection {
  RELEASE = 0,
  FORWARD = 1,
  BACKWARD = -1
};

enum class DirectionControl {
  PWM = 0,
  GPIO = 1
};

struct MotorPins {
  int in1_pin_;
  int in2_pin_;
  int pwm_pin_;
};

struct MotorConfig {
  std::string name;
  DirectionControl control_by_;
  MotorPins motor_pins_;
};

class Motor {
public:
  Motor(
    PWM& pwm,
    MotorConfig& motor_config);

  void Run(double velocity = 0);

private:
  PWM& pwm_;
  MotorConfig& motor_config_;
  MotorDirection direction_;

  void SetupByGPIO();
  void SetDirectionByGPIO();
  void SetDirectionByPWM();
  int GetPWMValueFromVelocity(double velocity);
  MotorDirection GetDirectionFromVelocity(double velocity, int pwm_value);
};

} // namespace devices
