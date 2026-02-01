#pragma once

#include "control/control_state.h"
#include "control/flight_mode.h"

FlightModeController &selectModeController(FlightMode mode,
                                           FlightModeController &stabilize,
                                           FlightModeController &constant_mode,
                                           FlightModeController &acro);
