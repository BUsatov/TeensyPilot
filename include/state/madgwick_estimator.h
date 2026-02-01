#pragma once

#include "state/state_estimator.h"
#include "imu/madgwick_filter.h"

class MadgwickEstimator final : public StateEstimator {
public:
  explicit MadgwickEstimator(float beta = 0.1f);

  void reset() override;
  void update(float gx_dps, float gy_dps, float gz_dps,
              float ax_g, float ay_g, float az_g,
              float dt_s) override;
  StateEstimate state() const override;

private:
  MadgwickFilter filter_;
  StateEstimate state_;
};
