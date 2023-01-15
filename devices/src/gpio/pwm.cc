#include "devices/gpio/pwm.h"
#include "devices/gpio/i2c.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>

namespace devices {

PWM::PWM(int i2c_address) {
  i2c_ = std::make_shared<I2C>(1, i2c_address);
  setAllPWM(0, 0);
  i2c_.get()->write8(__MODE2, __OUTDRV);
  i2c_.get()->write8(__MODE1, __ALLCALL);
  
  // wait for oscillator
  std::this_thread::sleep_for (std::chrono::milliseconds(5));

  int32_t mode1 = i2c_.get()->readU8(__MODE1);
  mode1 = mode1 & ~__SLEEP;  // wake up (reset sleep)
  i2c_.get()->write8(__MODE1, mode1);
  
  // wait for oscillator
  std::this_thread::sleep_for (std::chrono::milliseconds(5));
  setPWMFreq(1600.0);
}

void PWM::setPWM(int channel, int on, int off) {
  std::cout << ">>>>>>>>>>>>>>>>>> setting PWM channel " << channel  << " on = " << on << " off = "<< off << std::endl;
  i2c_.get()->write8(__LED0_ON_L + 4 * channel, on & 0xFF);
  i2c_.get()->write8(__LED0_ON_H + 4 * channel, on >> 8);
  i2c_.get()->write8(__LED0_OFF_L + 4 * channel, off & 0xFF);
  i2c_.get()->write8(__LED0_OFF_H + 4 * channel, off >> 8);
}

void PWM::setPWMFreq(double freq) {
  double prescaleval = 25000000.0;  // 25MHz
  prescaleval /= 4096.0;  // 12-bit
  prescaleval /= freq;
  prescaleval -= 1.0;
  if (debug_) {
    std::cout << "Setting PWM frequency to " << freq << std::endl;
    std::cout << "Estimated pre-scale: " << prescaleval << std::endl;
  }
  
  double prescale = floor(prescaleval + 0.5);
  if (debug_) {
    std::cout << "Final pre-scale: " << prescale << std::endl;
  }

  int oldmode = i2c_.get()->readU8(__MODE1);
  int newmode = (oldmode & 0x7F) | 0x10; // sleep
  i2c_.get()->write8(__MODE1, newmode); // go to sleep
  i2c_.get()->write8(__PRESCALE, int(floor(prescale)));
  i2c_.get()->write8(__MODE1, oldmode);
  
  // wait for oscillator
  std::this_thread::sleep_for (std::chrono::milliseconds(5));
  i2c_.get()->write8(__MODE1, oldmode | 0x80);
}

void PWM::setAllPWM(int on, int off) {
  i2c_.get()->write8(__ALL_LED_ON_L, on & 0xFF);
  i2c_.get()->write8(__ALL_LED_ON_H, on >> 8);
  i2c_.get()->write8(__ALL_LED_OFF_L, off & 0xFF);
  i2c_.get()->write8(__ALL_LED_OFF_H, off >> 8);
}

} // namespace devices
