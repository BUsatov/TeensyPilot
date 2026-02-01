#include "imu/comp_filter.h"

#include <math.h>

namespace {
constexpr float kRadToDeg = 57.2957795f;
}

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha_(alpha), roll_deg_(0.0f), pitch_deg_(0.0f), initialized_(false) {}

void ComplementaryFilter::reset() {
  roll_deg_ = 0.0f;
  pitch_deg_ = 0.0f;
  initialized_ = false;
}

void ComplementaryFilter::setAlpha(float alpha) {
  alpha_ = alpha;
}

void ComplementaryFilter::update(float gx_dps, float gy_dps, float gz_dps,
                                 float ax_g, float ay_g, float az_g,
                                 float dt) {
  if (dt <= 0.0f) {
    return;
  }

  float roll_acc = atan2f(ay_g, az_g) * kRadToDeg;
  float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * kRadToDeg;

  if (!initialized_) {
    roll_deg_ = roll_acc;
    pitch_deg_ = pitch_acc;
    initialized_ = true;
    return;
  }

  float roll_gyro = roll_deg_ + gx_dps * dt;
  float pitch_gyro = pitch_deg_ + gy_dps * dt;

  roll_deg_ = alpha_ * roll_gyro + (1.0f - alpha_) * roll_acc;
  pitch_deg_ = alpha_ * pitch_gyro + (1.0f - alpha_) * pitch_acc;

  (void)gz_dps;
}
