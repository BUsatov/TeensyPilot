#pragma once

#include "state/state_estimator.h"
#include "state/comp_filter.h"

class ComplementaryEstimator final : public StateEstimator {
public:
  explicit ComplementaryEstimator(float alpha = 0.98f);

  void reset() override;
  void update(float gx_dps, float gy_dps, float gz_dps,
              float ax_g, float ay_g, float az_g,
              float dt_s) override;
  StateEstimate state() const override;

private:
  ComplementaryFilter filter_;
  StateEstimate state_;
};
