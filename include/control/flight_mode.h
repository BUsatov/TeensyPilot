#pragma once

#include <stdint.h>

#include "control/control_state.h"
#include "hal/motor_mixer.h"
#include "rc/rc_receiver.h"
#include "state/state_estimator.h"

class FlightModeController {
public:
  explicit FlightModeController(MotorMixer &mixer) : mixer_(mixer) {}
  virtual ~FlightModeController() = default;
  virtual void apply(const RcChannels &rc,
                     const StateEstimate &state,
                     float dt_s) = 0;
  uint16_t lastThrottleUs() const { return last_throttle_us_; }
  uint16_t lastMotorUs() const { return last_motor_us_; }

protected:
  MotorMixer &mixer_;
  uint16_t last_throttle_us_ = 1000;
  uint16_t last_motor_us_ = 1000;
  uint16_t baseMotorUs(uint16_t throttle_us, uint16_t idle_pwm_us) const {
    if (throttle_us < idle_pwm_us) {
      throttle_us = idle_pwm_us;
    }
    return throttle_us;
  }
};
