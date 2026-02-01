#pragma once

#include "state/state_estimator.h"
#include "control/pid.h"

struct AttitudeSetpoint {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
};

struct RateSetpoint {
  float roll_dps;
  float pitch_dps;
  float yaw_dps;
};

struct RateOutput {
  float roll_us;
  float pitch_us;
  float yaw_us;
};

class AttitudeController {
public:
  virtual ~AttitudeController() = default;
  virtual void reset() = 0;
  virtual RateSetpoint update(const AttitudeSetpoint &sp,
                              const StateEstimate &state,
                              float dt_s) = 0;
};

class RateController {
public:
  virtual ~RateController() = default;
  virtual void reset() = 0;
  virtual RateOutput update(const RateSetpoint &sp,
                            float roll_rate_dps,
                            float pitch_rate_dps,
                            float yaw_rate_dps,
                            float dt_s) = 0;
};

class AttitudePController final : public AttitudeController {
public:
  AttitudePController(float kp_roll, float kp_pitch, float kp_yaw);

  void reset() override;
  RateSetpoint update(const AttitudeSetpoint &sp,
                      const StateEstimate &state,
                      float dt_s) override;

private:
  float kp_roll_;
  float kp_pitch_;
  float kp_yaw_;
};

class RatePidController final : public RateController {
public:
  RatePidController(float kp_roll, float kp_pitch, float kp_yaw,
                    float ki_roll, float ki_pitch, float ki_yaw,
                    float kd_roll, float kd_pitch, float kd_yaw,
                    float max_out_us);

  void reset() override;
  RateOutput update(const RateSetpoint &sp,
                    float roll_rate_dps,
                    float pitch_rate_dps,
                    float yaw_rate_dps,
                    float dt_s) override;

private:
  float max_out_us_;
  PidController roll_;
  PidController pitch_;
  PidController yaw_;
};
