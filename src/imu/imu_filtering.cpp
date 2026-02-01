#include "imu/imu_filtering.h"

#include <math.h>

#include "imu/imu_filters.h"
#include "imu/imu.h"

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

ImuFilterBank::ImuFilterBank()
    : throttle_norm_(0.0f),
      gyro_lpf_alpha_(0.0f),
      accel_lpf_alpha_(0.0f),
      gyro_lpf_enabled_(false),
      accel_lpf_enabled_(false),
      gyro_x_lpf_(0.0f),
      gyro_y_lpf_(0.0f),
      gyro_z_lpf_(0.0f),
      accel_x_lpf_(0.0f),
      accel_y_lpf_(0.0f),
      accel_z_lpf_(0.0f),
      fs_hz_(0.0f) {
  gyro_notch_[0].configure(IMU_NOTCH1_GYRO_HZ, IMU_NOTCH1_GYRO_Q, IMU_NOTCH_GYRO_SCALE);
  gyro_notch_[1].configure(IMU_NOTCH2_GYRO_HZ, IMU_NOTCH2_GYRO_Q, IMU_NOTCH_GYRO_SCALE);
  gyro_notch_[2].configure(IMU_NOTCH3_GYRO_HZ, IMU_NOTCH3_GYRO_Q, IMU_NOTCH_GYRO_SCALE);

  accel_notch_[0].configure(IMU_NOTCH1_ACCEL_HZ, IMU_NOTCH1_ACCEL_Q, IMU_NOTCH_ACCEL_SCALE);
  accel_notch_[1].configure(IMU_NOTCH2_ACCEL_HZ, IMU_NOTCH2_ACCEL_Q, IMU_NOTCH_ACCEL_SCALE);
  accel_notch_[2].configure(IMU_NOTCH3_ACCEL_HZ, IMU_NOTCH3_ACCEL_Q, IMU_NOTCH_ACCEL_SCALE);

  gyro_lpf_enabled_ = IMU_LPF_GYRO_HZ > 0.0f;
  accel_lpf_enabled_ = IMU_LPF_ACCEL_HZ > 0.0f;
}

void ImuFilterBank::updateThrottleUs(uint16_t throttle_us) {
  float t = static_cast<float>(throttle_us);
  float min_t = IMU_NOTCH_THROTTLE_MIN_US;
  float max_t = IMU_NOTCH_THROTTLE_MAX_US;
  if (max_t <= min_t) {
    throttle_norm_ = 0.0f;
  } else {
    throttle_norm_ = clampf((t - min_t) / (max_t - min_t), 0.0f, 1.0f);
  }
}

void ImuFilterBank::updateFs(float dt_s) {
  if (dt_s <= 0.0f) {
    return;
  }
  fs_hz_ = 1.0f / dt_s;

  if (gyro_lpf_enabled_) {
    float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * IMU_LPF_GYRO_HZ);
    gyro_lpf_alpha_ = dt_s / (dt_s + rc);
  }
  if (accel_lpf_enabled_) {
    float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * IMU_LPF_ACCEL_HZ);
    accel_lpf_alpha_ = dt_s / (dt_s + rc);
  }
}

void ImuFilterBank::updateNotches() {
  if (fs_hz_ <= 0.0f) {
    return;
  }
  for (int i = 0; i < 3; i++) {
    gyro_notch_[i].updateCoeffs(fs_hz_, throttle_norm_);
    accel_notch_[i].updateCoeffs(fs_hz_, throttle_norm_);
  }
}

void ImuFilterBank::apply(ImuSample &sample, float dt_s) {
  updateFs(dt_s);
  updateNotches();

  float gx = sample.gx_dps;
  float gy = sample.gy_dps;
  float gz = sample.gz_dps;

  float ax = sample.ax_g;
  float ay = sample.ay_g;
  float az = sample.az_g;

  for (int i = 0; i < 3; i++) {
    if (gyro_notch_[i].enabled) {
      gx = gyro_notch_[i].process(gx);
      gy = gyro_notch_[i].process(gy);
      gz = gyro_notch_[i].process(gz);
    }
    if (accel_notch_[i].enabled) {
      ax = accel_notch_[i].process(ax);
      ay = accel_notch_[i].process(ay);
      az = accel_notch_[i].process(az);
    }
  }

  if (gyro_lpf_enabled_) {
    gyro_x_lpf_ += gyro_lpf_alpha_ * (gx - gyro_x_lpf_);
    gyro_y_lpf_ += gyro_lpf_alpha_ * (gy - gyro_y_lpf_);
    gyro_z_lpf_ += gyro_lpf_alpha_ * (gz - gyro_z_lpf_);
    gx = gyro_x_lpf_;
    gy = gyro_y_lpf_;
    gz = gyro_z_lpf_;
  }

  if (accel_lpf_enabled_) {
    accel_x_lpf_ += accel_lpf_alpha_ * (ax - accel_x_lpf_);
    accel_y_lpf_ += accel_lpf_alpha_ * (ay - accel_y_lpf_);
    accel_z_lpf_ += accel_lpf_alpha_ * (az - accel_z_lpf_);
    ax = accel_x_lpf_;
    ay = accel_y_lpf_;
    az = accel_z_lpf_;
  }

  sample.gx_dps = gx;
  sample.gy_dps = gy;
  sample.gz_dps = gz;
  sample.ax_g = ax;
  sample.ay_g = ay;
  sample.az_g = az;
}

void ImuFilterBank::Notch::reset() {
  z1 = 0.0f;
  z2 = 0.0f;
}

void ImuFilterBank::Notch::configure(float base_hz_in, float q_in, float scale_in) {
  base_hz = base_hz_in;
  q = q_in;
  scale = scale_in;
  enabled = (base_hz > 0.0f && q > 0.0f);
  reset();
}

void ImuFilterBank::Notch::updateCoeffs(float fs_hz, float throttle_norm) {
  if (!enabled || fs_hz <= 0.0f) {
    return;
  }
  float center = base_hz * (1.0f + scale * throttle_norm);
  if (center <= 0.0f || center >= (0.5f * fs_hz)) {
    return;
  }

  float w0 = 2.0f * static_cast<float>(M_PI) * center / fs_hz;
  float cos_w0 = cosf(w0);
  float sin_w0 = sinf(w0);
  float alpha = sin_w0 / (2.0f * q);

  float b0n = 1.0f;
  float b1n = -2.0f * cos_w0;
  float b2n = 1.0f;
  float a0n = 1.0f + alpha;
  float a1n = -2.0f * cos_w0;
  float a2n = 1.0f - alpha;

  b0 = b0n / a0n;
  b1 = b1n / a0n;
  b2 = b2n / a0n;
  a1 = a1n / a0n;
  a2 = a2n / a0n;
}

float ImuFilterBank::Notch::process(float x) {
  float y = b0 * x + z1;
  z1 = b1 * x - a1 * y + z2;
  z2 = b2 * x - a2 * y;
  return y;
}
