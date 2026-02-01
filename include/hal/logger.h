#pragma once

class Logger {
public:
  virtual ~Logger() = default;
  virtual void begin() = 0;
  virtual void logLine(const char *line) = 0;
};
