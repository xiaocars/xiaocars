#pragma once

namespace devices {
  namespace constants {

inline int constexpr LOW = 0;
inline int constexpr HIGH = 1;
inline int constexpr PWM_LOW = 0;
inline int constexpr PWM_HIGH = 4096;
inline int constexpr min_pwm_value = 60;  // Minimum speed for left motor
inline int constexpr max_pwm_value = 255;  // Maximum speed for left motor
inline double constexpr speed_tolerance = 1.0e-2; // Speed tolerance level
inline int constexpr MOTOR_K_GAIN = 16; // Value to amplify the PWM singal to correct levels

  } // namespace constants
} //namespace devices
