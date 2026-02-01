#include "hal/motor_mixer.h"

MotorMixer::MotorMixer(EscPwm &esc) : esc_(esc), order_{0, 1, 2, 3} {}

void MotorMixer::setOrder(const uint8_t order[EscPwm::kMotorCount]) {
  for (uint8_t i = 0; i < EscPwm::kMotorCount; i++) {
    order_[i] = order[i];
  }
}

void MotorMixer::setThrottleUs(uint16_t us) {
  for (uint8_t i = 0; i < EscPwm::kMotorCount; i++) {
    esc_.writeUs(order_[i], us);
    last_out_[order_[i]] = us;
  }
}

void MotorMixer::setMixUs(uint16_t throttle_us, int16_t roll_us, int16_t pitch_us, int16_t yaw_us) {
  // Quad X mixing: m0 FL, m1 FR, m2 RR, m3 RL (before order_ mapping)
  int32_t m0 = static_cast<int32_t>(throttle_us) + pitch_us + roll_us - yaw_us;
  int32_t m1 = static_cast<int32_t>(throttle_us) + pitch_us - roll_us + yaw_us;
  int32_t m2 = static_cast<int32_t>(throttle_us) - pitch_us - roll_us - yaw_us;
  int32_t m3 = static_cast<int32_t>(throttle_us) - pitch_us + roll_us + yaw_us;

  if (m0 < 0) m0 = 0;
  if (m1 < 0) m1 = 0;
  if (m2 < 0) m2 = 0;
  if (m3 < 0) m3 = 0;

  esc_.writeUs(order_[0], static_cast<uint16_t>(m0));
  esc_.writeUs(order_[1], static_cast<uint16_t>(m1));
  esc_.writeUs(order_[2], static_cast<uint16_t>(m2));
  esc_.writeUs(order_[3], static_cast<uint16_t>(m3));

  last_out_[order_[0]] = static_cast<uint16_t>(m0);
  last_out_[order_[1]] = static_cast<uint16_t>(m1);
  last_out_[order_[2]] = static_cast<uint16_t>(m2);
  last_out_[order_[3]] = static_cast<uint16_t>(m3);
}

void MotorMixer::getLastOutputs(uint16_t out[EscPwm::kMotorCount]) const {
  for (uint8_t i = 0; i < EscPwm::kMotorCount; i++) {
    out[i] = last_out_[i];
  }
}
