#include "control/flight_mode_factory.h"

FlightModeController &selectModeController(FlightMode mode,
                                           FlightModeController &stabilize,
                                           FlightModeController &constant_mode,
                                           FlightModeController &acro) {
  switch (mode) {
    case FlightMode::kStabilize:
      return stabilize;
    case FlightMode::kAcro:
      return acro;
    case FlightMode::kConstant:
    default:
      return constant_mode;
  }
}
