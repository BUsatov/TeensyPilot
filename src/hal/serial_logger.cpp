#include "hal/serial_logger.h"

SerialLogger::SerialLogger(Print &serial) : serial_(serial) {}

void SerialLogger::begin() {
}

void SerialLogger::logLine(const char *line) {
  serial_.println(line);
}
