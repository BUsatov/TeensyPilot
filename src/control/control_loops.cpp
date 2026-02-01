#include "control/control_loops.h"

static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

AttitudePController::AttitudePController(float kp_roll, float kp_pitch, float kp_yaw)
    : kp_roll_(kp_roll), kp_pitch_(kp_pitch), kp_yaw_(kp_yaw) {}

void AttitudePController::reset() {}

RateSetpoint AttitudePController::update(const AttitudeSetpoint &sp,
                                         const StateEstimate &state,
                                         float dt_s) {
  (void)dt_s;
  RateSetpoint out{};
  float roll_err = sp.roll_deg - state.roll_deg;
  float pitch_err = sp.pitch_deg - state.pitch_deg;
  float yaw_err = sp.yaw_deg - state.yaw_deg;

  out.roll_dps = kp_roll_ * roll_err;
  out.pitch_dps = kp_pitch_ * pitch_err;
  out.yaw_dps = kp_yaw_ * yaw_err;
  return out;
}

RatePidController::RatePidController(float kp_roll, float kp_pitch, float kp_yaw,
                                     float ki_roll, float ki_pitch, float ki_yaw,
                                     float kd_roll, float kd_pitch, float kd_yaw,
                                     float max_out_us)
    : max_out_us_(max_out_us),
      roll_(kp_roll, ki_roll, kd_roll, -max_out_us, max_out_us),
      pitch_(kp_pitch, ki_pitch, kd_pitch, -max_out_us, max_out_us),
      yaw_(kp_yaw, ki_yaw, kd_yaw, -max_out_us, max_out_us) {}

void RatePidController::reset() {
  roll_.reset();
  pitch_.reset();
  yaw_.reset();
}

RateOutput RatePidController::update(const RateSetpoint &sp,
                                     float roll_rate_dps,
                                     float pitch_rate_dps,
                                     float yaw_rate_dps,
                                     float dt_s) {
  RateOutput out{};

  float roll_err = sp.roll_dps - roll_rate_dps;
  float pitch_err = sp.pitch_dps - pitch_rate_dps;
  float yaw_err = sp.yaw_dps - yaw_rate_dps;

  out.roll_us = clampf(roll_.update(roll_err, dt_s), -max_out_us_, max_out_us_);
  out.pitch_us = clampf(pitch_.update(pitch_err, dt_s), -max_out_us_, max_out_us_);
  out.yaw_us = clampf(yaw_.update(yaw_err, dt_s), -max_out_us_, max_out_us_);
  return out;
}
