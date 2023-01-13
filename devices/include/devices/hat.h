#pragma once

#include <devices/motor.h>
#include <devices/gpio/pwm.h>

namespace devices {

class HAT {
public:
  HAT(int i2c_address, double frequency);

  void RunRightMotor(int velocity);
  void RunLeftMotor(int velocity);

private:
  int i2c_address_ = 0x60;
  double frequency_ = 1600.0;

  int right_motor_in1_pin = 10;
  int right_motor_in2_pin = 9;
  int right_motor_pwm_pin = 8;
  
  PWM pwm_;
  Motor left_motor_;
  Motor right_motor_;
};

} // namespace devices
