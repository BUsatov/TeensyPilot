#include "state/madgwick_filter.h"

#include <math.h>

namespace {
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToDeg = 57.2957795f;
}

MadgwickFilter::MadgwickFilter(float beta)
    : beta_(beta), q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f), initialized_(false) {}

void MadgwickFilter::reset() {
  q0_ = 1.0f;
  q1_ = 0.0f;
  q2_ = 0.0f;
  q3_ = 0.0f;
  initialized_ = false;
}

void MadgwickFilter::setBeta(float beta) {
  beta_ = beta;
}

void MadgwickFilter::update(float gx_dps, float gy_dps, float gz_dps,
                            float ax_g, float ay_g, float az_g,
                            float dt) {
  if (dt <= 0.0f) {
    return;
  }

  float gx = gx_dps * kDegToRad;
  float gy = gy_dps * kDegToRad;
  float gz = gz_dps * kDegToRad;

  float ax = ax_g;
  float ay = ay_g;
  float az = az_g;

  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm <= 0.0f) {
    return;
  }
  ax /= norm;
  ay /= norm;
  az /= norm;

  float q0 = q0_;
  float q1 = q1_;
  float q2 = q2_;
  float q3 = q3_;

  // Gradient descent algorithm corrective step (IMU-only version)
  float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
  float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
  float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

  float s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
  float s1 = 2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
  float s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
  float s3 = 2.0f * q1 * f1 + 2.0f * q2 * f2;

  float s_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (s_norm > 0.0f) {
    s0 /= s_norm;
    s1 /= s_norm;
    s2 /= s_norm;
    s3 /= s_norm;
  }

  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta_ * s0;
  float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta_ * s1;
  float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta_ * s2;
  float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta_ * s3;

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (q_norm > 0.0f) {
    q0 /= q_norm;
    q1 /= q_norm;
    q2 /= q_norm;
    q3 /= q_norm;
  }

  q0_ = q0;
  q1_ = q1;
  q2_ = q2;
  q3_ = q3;
  initialized_ = true;
}

void MadgwickFilter::getEulerDeg(float &roll, float &pitch, float &yaw) const {
  float q0 = q0_;
  float q1 = q1_;
  float q2 = q2_;
  float q3 = q3_;

  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  roll = atan2f(sinr_cosp, cosr_cosp) * kRadToDeg;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(90.0f, sinp);
  } else {
    pitch = asinf(sinp) * kRadToDeg;
  }

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  yaw = atan2f(siny_cosp, cosy_cosp) * kRadToDeg;
}
