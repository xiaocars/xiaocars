#include <devices/hat.h>
#include <devices/motor.h>
#include <devices/gpio/pwm.h>

namespace devices {

HAT::HAT(int i2c_address, double frequency)
  : i2c_address_(i2c_address),
  frequency_(frequency),
  pwm_(PWM(i2c_address)),
  left_motor_(Motor("left", pwm_, 10, 9, 8, DirectionControl::PWM)),
  right_motor_(Motor("right", pwm_, 33, 31, 13, DirectionControl::GPIO)){}

void HAT::RunRightMotor(int velocity) {
  right_motor_.Run(velocity);
}

void HAT::RunLeftMotor(int velocity) {
  left_motor_.Run(velocity);
}

} // namespace devices
