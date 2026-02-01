#include "control/pid.h"

static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

PidController::PidController(float kp, float ki, float kd, float out_min, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max),
      i_min_(out_min), i_max_(out_max),
      integrator_(0.0f), prev_error_(0.0f), has_prev_(false) {}

PidController::PidController(float kp, float ki, float kd, float out_min, float out_max,
                             float i_min, float i_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max),
      i_min_(i_min), i_max_(i_max),
      integrator_(0.0f), prev_error_(0.0f), has_prev_(false) {}

void PidController::reset() {
  integrator_ = 0.0f;
  prev_error_ = 0.0f;
  has_prev_ = false;
}

void PidController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PidController::setIntegratorLimits(float i_min, float i_max) {
  i_min_ = i_min;
  i_max_ = i_max;
}

float PidController::update(float error, float dt_s) {
  if (dt_s <= 0.0f) {
    return 0.0f;
  }

  float p = kp_ * error;
  integrator_ += error * dt_s;
  integrator_ = clampf(integrator_, i_min_, i_max_);
  float i = ki_ * integrator_;

  float d = 0.0f;
  if (has_prev_) {
    d = kd_ * ((error - prev_error_) / dt_s);
  }
  prev_error_ = error;
  has_prev_ = true;

  float out = p + i + d;
  return clampf(out, out_min_, out_max_);
}
