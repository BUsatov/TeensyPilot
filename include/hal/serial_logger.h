#pragma once

#include <Arduino.h>

#include "hal/logger.h"

class SerialLogger final : public Logger {
public:
  explicit SerialLogger(Print &serial);

  void begin() override;
  void logLine(const char *line) override;

private:
  Print &serial_;
};
