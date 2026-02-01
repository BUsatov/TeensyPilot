#pragma once

#include <stdint.h>

struct ImuSample;

class ImuFilterBank {
public:
  ImuFilterBank();

  void updateThrottleUs(uint16_t throttle_us);
  void apply(ImuSample &sample, float dt_s);

private:
  float throttle_norm_;

  float gyro_lpf_alpha_;
  float accel_lpf_alpha_;
  bool gyro_lpf_enabled_;
  bool accel_lpf_enabled_;

  float gyro_x_lpf_;
  float gyro_y_lpf_;
  float gyro_z_lpf_;
  float accel_x_lpf_;
  float accel_y_lpf_;
  float accel_z_lpf_;

  struct Notch {
    float base_hz;
    float q;
    float scale;
    bool enabled;

    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    float z1;
    float z2;

    void reset();
    void configure(float base_hz, float q, float scale);
    void updateCoeffs(float fs_hz, float throttle_norm);
    float process(float x);
  };

  Notch gyro_notch_[3];
  Notch accel_notch_[3];

  float fs_hz_;
  void updateFs(float dt_s);
  void updateNotches();
};
