#include "control/stabilize_mode.h"

#include "control/control_state.h"
#include "control/control_config.h"
#include "control/control_loops.h"
#include "control/control_gains.h"

StabilizeMode::StabilizeMode(MotorMixer &mixer) : FlightModeController(mixer) {}

void StabilizeMode::apply(const RcChannels &rc,
                          const StateEstimate &state,
                          float dt_s) {
  static AttitudePController attitude_ctrl(ATT_P_ROLL, ATT_P_PITCH, ATT_P_YAW);
  static RatePidController rate_ctrl(RATE_P_ROLL, RATE_P_PITCH, RATE_P_YAW,
                                     RATE_I_ROLL, RATE_I_PITCH, RATE_I_YAW,
                                     RATE_D_ROLL, RATE_D_PITCH, RATE_D_YAW,
                                     RATE_MAX_US);

  float roll_cmd = (static_cast<float>(rc.ch[RcChannels::kRoll]) - 1500.0f) / 500.0f;
  float pitch_cmd = (static_cast<float>(rc.ch[RcChannels::kPitch]) - 1500.0f) / 500.0f;
  float yaw_cmd = (static_cast<float>(rc.ch[RcChannels::kYaw]) - 1500.0f) / 500.0f;

  if (roll_cmd > 1.0f) roll_cmd = 1.0f;
  if (roll_cmd < -1.0f) roll_cmd = -1.0f;
  if (pitch_cmd > 1.0f) pitch_cmd = 1.0f;
  if (pitch_cmd < -1.0f) pitch_cmd = -1.0f;
  if (yaw_cmd > 1.0f) yaw_cmd = 1.0f;
  if (yaw_cmd < -1.0f) yaw_cmd = -1.0f;

  static constexpr float kMaxAngleDeg = ATT_MAX_ANGLE_DEG;
  static constexpr float kMaxYawRateDps = ATT_MAX_YAW_RATE_DPS;

  AttitudeSetpoint att_sp{};
  att_sp.roll_deg = roll_cmd * kMaxAngleDeg;
  att_sp.pitch_deg = pitch_cmd * kMaxAngleDeg;
  att_sp.yaw_deg = state.yaw_deg;

  RateSetpoint rate_sp = attitude_ctrl.update(att_sp, state, dt_s);
  rate_sp.yaw_dps = yaw_cmd * kMaxYawRateDps;

  RateOutput rate_out = rate_ctrl.update(rate_sp,
                                         state.roll_rate_dps,
                                         state.pitch_rate_dps,
                                         state.yaw_rate_dps,
                                         dt_s);

  debug_.att_sp_roll_deg = att_sp.roll_deg;
  debug_.att_sp_pitch_deg = att_sp.pitch_deg;
  debug_.att_sp_yaw_deg = att_sp.yaw_deg;
  debug_.rate_sp_roll_dps = rate_sp.roll_dps;
  debug_.rate_sp_pitch_dps = rate_sp.pitch_dps;
  debug_.rate_sp_yaw_dps = rate_sp.yaw_dps;
  debug_.rate_meas_roll_dps = state.roll_rate_dps;
  debug_.rate_meas_pitch_dps = state.pitch_rate_dps;
  debug_.rate_meas_yaw_dps = state.yaw_rate_dps;
  debug_.rate_out_roll_us = rate_out.roll_us;
  debug_.rate_out_pitch_us = rate_out.pitch_us;
  debug_.rate_out_yaw_us = rate_out.yaw_us;

  last_throttle_us_ = rc.ch[RcChannels::kThrottle];
  last_motor_us_ = baseMotorUs(last_throttle_us_, kIdlePwmUs);
  mixer_.setMixUs(last_motor_us_,
                  static_cast<int16_t>(rate_out.roll_us),
                  static_cast<int16_t>(rate_out.pitch_us),
                  static_cast<int16_t>(rate_out.yaw_us));
}
