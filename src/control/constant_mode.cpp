#include "control/constant_mode.h"

#include "control/control_state.h"
#include "control/control_config.h"

ConstantMode::ConstantMode(MotorMixer &mixer) : FlightModeController(mixer) {}

void ConstantMode::apply(const RcChannels &rc,
                         const StateEstimate &state,
                         float dt_s) {
  (void)state;
  (void)dt_s;
  last_throttle_us_ = rc.ch[RcChannels::kThrottle];
  last_motor_us_ = baseMotorUs(last_throttle_us_, kIdlePwmUs);
  mixer_.setThrottleUs(last_motor_us_);
}
