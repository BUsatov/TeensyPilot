#include "state/comp_estimator.h"

ComplementaryEstimator::ComplementaryEstimator(float alpha)
    : filter_(alpha), state_{} {}

void ComplementaryEstimator::reset() {
  filter_.reset();
  state_ = {};
}

void ComplementaryEstimator::update(float gx_dps, float gy_dps, float gz_dps,
                                   float ax_g, float ay_g, float az_g,
                                   float dt_s) {
  filter_.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt_s);

  state_.roll_deg = filter_.rollDeg();
  state_.pitch_deg = filter_.pitchDeg();
  state_.yaw_deg = 0.0f; // No yaw estimate without mag/GNSS.
  state_.roll_rate_dps = 0.0f;
  state_.pitch_rate_dps = 0.0f;
  state_.yaw_rate_dps = 0.0f;

  // Position/velocity/altitude require additional sensors (e.g., GPS/baro) and fusion.
  state_.pos_x_m = 0.0f;
  state_.pos_y_m = 0.0f;
  state_.pos_z_m = 0.0f;
  state_.vel_x_mps = 0.0f;
  state_.vel_y_mps = 0.0f;
  state_.vel_z_mps = 0.0f;
  state_.alt_m = 0.0f;
}

StateEstimate ComplementaryEstimator::state() const {
  return state_;
}
