#pragma once

#include <stdint.h>

#include "rc/rc_receiver.h"

enum class ArmState : uint8_t {
  kDisarmed = 0,
  kArmed = 1,
};

enum class FlightMode : uint8_t {
  kStabilize = 0,
  kConstant = 1,
  kAcro = 2,
};

struct ControlState {
  ArmState arm;
  FlightMode mode;
};

void updateControlStateFromRc(ControlState &state, const RcChannels &ch);
