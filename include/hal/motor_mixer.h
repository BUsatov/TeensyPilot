#pragma once

#include <stdint.h>

#include "hal/esc_pwm.h"

class MotorMixer {
public:
  explicit MotorMixer(EscPwm &esc);

  void setOrder(const uint8_t order[EscPwm::kMotorCount]);
  void setThrottleUs(uint16_t us);
  void setMixUs(uint16_t throttle_us, int16_t roll_us, int16_t pitch_us, int16_t yaw_us);
  void getLastOutputs(uint16_t out[EscPwm::kMotorCount]) const;

private:
  EscPwm &esc_;
  uint8_t order_[EscPwm::kMotorCount];
  uint16_t last_out_[EscPwm::kMotorCount] = {0, 0, 0, 0};
};
