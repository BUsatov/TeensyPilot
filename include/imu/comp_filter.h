#pragma once

#include <stdint.h>

class ComplementaryFilter {
public:
  explicit ComplementaryFilter(float alpha = 0.98f);

  void reset();
  void setAlpha(float alpha);

  // gyro_dps: degrees per second; accel_g: g units; dt: seconds.
  void update(float gx_dps, float gy_dps, float gz_dps,
              float ax_g, float ay_g, float az_g,
              float dt);

  float rollDeg() const { return roll_deg_; }
  float pitchDeg() const { return pitch_deg_; }

private:
  float alpha_;
  float roll_deg_;
  float pitch_deg_;
  bool initialized_;
};
