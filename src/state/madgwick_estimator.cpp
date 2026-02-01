#include "state/madgwick_estimator.h"

MadgwickEstimator::MadgwickEstimator(float beta)
    : filter_(beta), state_{} {}

void MadgwickEstimator::reset() {
  filter_.reset();
  state_ = {};
}

void MadgwickEstimator::update(float gx_dps, float gy_dps, float gz_dps,
                               float ax_g, float ay_g, float az_g,
                               float dt_s) {
  filter_.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt_s);

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  filter_.getEulerDeg(roll, pitch, yaw);

  state_.roll_deg = roll;
  state_.pitch_deg = pitch;
  state_.yaw_deg = yaw;
  state_.roll_rate_dps = 0.0f;
  state_.pitch_rate_dps = 0.0f;
  state_.yaw_rate_dps = 0.0f;

  state_.pos_x_m = 0.0f;
  state_.pos_y_m = 0.0f;
  state_.pos_z_m = 0.0f;
  state_.vel_x_mps = 0.0f;
  state_.vel_y_mps = 0.0f;
  state_.vel_z_mps = 0.0f;
  state_.alt_m = 0.0f;
}

StateEstimate MadgwickEstimator::state() const {
  return state_;
}
