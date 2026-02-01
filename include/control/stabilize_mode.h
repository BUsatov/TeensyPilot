#pragma once

#include "control/flight_mode.h"

class StabilizeMode final : public FlightModeController {
public:
  explicit StabilizeMode(MotorMixer &mixer);
  void apply(const RcChannels &rc,
             const StateEstimate &state,
             float dt_s) override;

  struct Debug {
    float att_sp_roll_deg;
    float att_sp_pitch_deg;
    float att_sp_yaw_deg;
    float rate_sp_roll_dps;
    float rate_sp_pitch_dps;
    float rate_sp_yaw_dps;
    float rate_meas_roll_dps;
    float rate_meas_pitch_dps;
    float rate_meas_yaw_dps;
    float rate_out_roll_us;
    float rate_out_pitch_us;
    float rate_out_yaw_us;
  };

  const Debug &debug() const { return debug_; }

private:
  Debug debug_{};
};
