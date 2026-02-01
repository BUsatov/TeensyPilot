#pragma once

class MadgwickFilter {
public:
  explicit MadgwickFilter(float beta = 0.1f);

  void reset();
  void setBeta(float beta);

  // gyro_dps: degrees per second; accel_g: g units; dt: seconds.
  void update(float gx_dps, float gy_dps, float gz_dps,
              float ax_g, float ay_g, float az_g,
              float dt);

  void getEulerDeg(float &roll, float &pitch, float &yaw) const;

private:
  float beta_;
  float q0_;
  float q1_;
  float q2_;
  float q3_;
  bool initialized_;
};
