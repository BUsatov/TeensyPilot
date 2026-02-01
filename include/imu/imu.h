#pragma once

#include <stdint.h>

struct ImuSample {
  int16_t ax_raw;
  int16_t ay_raw;
  int16_t az_raw;
  int16_t gx_raw;
  int16_t gy_raw;
  int16_t gz_raw;
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
};

class Imu {
public:
  virtual ~Imu() = default;

  virtual bool begin() = 0;
  virtual bool calibrate(uint16_t samples, uint16_t delay_ms) = 0;
  virtual bool read(ImuSample &sample) = 0;
};
