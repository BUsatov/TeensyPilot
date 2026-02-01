#include "hal/esc_pwm.h"

EscPwm::EscPwm(const uint8_t pins[kMotorCount])
    : pins_{pins[0], pins[1], pins[2], pins[3]},
      min_us_(1000),
      max_us_(2000) {}

void EscPwm::begin(uint16_t min_us, uint16_t max_us) {
  min_us_ = min_us;
  max_us_ = max_us;
  for (uint8_t i = 0; i < kMotorCount; i++) {
    servos_[i].attach(pins_[i], min_us_, max_us_);
  }
}

void EscPwm::writeUs(uint8_t motor_index, uint16_t us) {
  if (motor_index >= kMotorCount) {
    return;
  }
  if (us < min_us_) {
    us = min_us_;
  } else if (us > max_us_) {
    us = max_us_;
  }
  servos_[motor_index].writeMicroseconds(us);
}

void EscPwm::writeAllUs(uint16_t us) {
  for (uint8_t i = 0; i < kMotorCount; i++) {
    writeUs(i, us);
  }
}
