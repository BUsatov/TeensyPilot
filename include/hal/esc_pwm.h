#pragma once

#include <Arduino.h>
#include <Servo.h>

class EscPwm {
public:
  static constexpr uint8_t kMotorCount = 4;

  explicit EscPwm(const uint8_t pins[kMotorCount]);

  void begin(uint16_t min_us = 1000, uint16_t max_us = 2000);
  void writeUs(uint8_t motor_index, uint16_t us);
  void writeAllUs(uint16_t us);

private:
  uint8_t pins_[kMotorCount];
  Servo servos_[kMotorCount];
  uint16_t min_us_;
  uint16_t max_us_;
};
