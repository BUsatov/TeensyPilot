#include "control/control_state.h"

namespace {
constexpr uint8_t kArmChannel = 4;     // RC9 (0-based index)
constexpr uint16_t kArmThreshold = 1500;
constexpr uint8_t kModeChannel = 5;    // RC10 (0-based index)
constexpr uint16_t kModeThreshold = 1500;
}

void updateControlStateFromRc(ControlState &state, const RcChannels &ch) {
  state.arm = (ch.ch[kArmChannel] >= kArmThreshold) ? ArmState::kArmed
                                                    : ArmState::kDisarmed;
  uint16_t mode_raw = ch.ch[kModeChannel];
  if (mode_raw < 1200) {
    state.mode = FlightMode::kStabilize;
  } else if (mode_raw < 1600) {
    state.mode = FlightMode::kConstant;
  } else {
    state.mode = FlightMode::kAcro;
  }
}
