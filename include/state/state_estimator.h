#pragma once

struct StateEstimate {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float roll_rate_dps;
  float pitch_rate_dps;
  float yaw_rate_dps;

  float pos_x_m;
  float pos_y_m;
  float pos_z_m;

  float vel_x_mps;
  float vel_y_mps;
  float vel_z_mps;

  float alt_m;
};

class StateEstimator {
public:
  virtual ~StateEstimator() = default;

  virtual void reset() = 0;
  virtual void update(float gx_dps, float gy_dps, float gz_dps,
                      float ax_g, float ay_g, float az_g,
                      float dt_s) = 0;
  virtual StateEstimate state() const = 0;
};
