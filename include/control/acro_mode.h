#pragma once

#include "control/flight_mode.h"

class AcroMode final : public FlightModeController {
public:
  explicit AcroMode(MotorMixer &mixer);
  void apply(const RcChannels &rc,
             const StateEstimate &state,
             float dt_s) override;
};
