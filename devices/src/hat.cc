#include <devices/hat.h>
#include <devices/motor.h>
#include <devices/gpio/pwm.h>

#include <iostream>
#include <string>

namespace devices {

// HAT::HAT(PWM& pwm,
//   MotorConfig& left_motor_config,
//   MotorConfig& right_motor_config)
//   : pwm_{pwm} {
    
//   left_motor_ = CreateMotor(left_motor_config);
//   right_motor_ = CreateMotor(right_motor_config);
// }


HAT::HAT(PWM pwm,
  MotorConfig left_motor_config,
  MotorConfig right_motor_config)
  : pwm_{pwm} {
  left_motor_ = CreateMotor(left_motor_config); 
  right_motor_ = CreateMotor(right_motor_config);
}

std::shared_ptr<Motor> HAT::CreateMotor(MotorConfig motor_config) {
  std::cout << "???????????????????????????????????????????????????????????????????????????" << std::endl;
  std::cout << "Creating Motor " << motor_config.name << " with pwm pin" << motor_config.motor_pins_.pwm_pin_ << std::endl;
  return std::move(std::make_shared<Motor>(pwm_, motor_config));
}

void HAT::RunRightMotor(double velocity) {
  right_motor_->Run(velocity);
}

void HAT::RunLeftMotor(double velocity) {
  left_motor_->Run(velocity);
}

} // namespace devices
