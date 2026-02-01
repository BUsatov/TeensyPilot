#pragma once

class PidController {
public:
  PidController(float kp, float ki, float kd, float out_min, float out_max);
  PidController(float kp, float ki, float kd, float out_min, float out_max,
                float i_min, float i_max);

  void reset();
  void setGains(float kp, float ki, float kd);
  float update(float error, float dt_s);
  void setIntegratorLimits(float i_min, float i_max);

private:
  float kp_;
  float ki_;
  float kd_;
  float out_min_;
  float out_max_;
  float i_min_;
  float i_max_;
  float integrator_;
  float prev_error_;
  bool has_prev_;
};
