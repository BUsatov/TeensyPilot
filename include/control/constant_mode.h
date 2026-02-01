#pragma once

#include "control/flight_mode.h"

class ConstantMode final : public FlightModeController {
public:
  explicit ConstantMode(MotorMixer &mixer);
  void apply(const RcChannels &rc,
             const StateEstimate &state,
             float dt_s) override;
};
