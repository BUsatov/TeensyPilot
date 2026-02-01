#include "control/acro_mode.h"

#include "control/control_state.h"
#include "control/control_config.h"

AcroMode::AcroMode(MotorMixer &mixer) : FlightModeController(mixer) {}

void AcroMode::apply(const RcChannels &rc,
                     const StateEstimate &state,
                     float dt_s) {
  (void)state;
  (void)dt_s;
  // TODO: replace with actual acro rate controller/mixer.
  last_throttle_us_ = rc.ch[RcChannels::kThrottle];
  last_motor_us_ = baseMotorUs(last_throttle_us_, kIdlePwmUs);
  mixer_.setThrottleUs(last_motor_us_);
}
