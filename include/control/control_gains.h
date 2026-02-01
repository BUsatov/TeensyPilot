#pragma once

// Attitude PID gains (deg -> rate setpoint)
#define ATT_P_ROLL  1.0f
#define ATT_I_ROLL  0.0f
#define ATT_D_ROLL  0.0f

#define ATT_P_PITCH 1.0f
#define ATT_I_PITCH 0.0f
#define ATT_D_PITCH 0.0f

#define ATT_P_YAW   1.0f
#define ATT_I_YAW   0.0f
#define ATT_D_YAW   0.0f

// Rate PID gains (dps -> motor mix us)
#define RATE_P_ROLL  1.2f
#define RATE_I_ROLL  0.1f
#define RATE_D_ROLL  0.005f

#define RATE_P_PITCH 1.2f
#define RATE_I_PITCH 0.1f
#define RATE_D_PITCH 0.005f

#define RATE_P_YAW   1.8f
#define RATE_I_YAW   0.03f
#define RATE_D_YAW   0.03f

// Limits
#define RATE_MAX_US  300.0f
#define ATT_MAX_ANGLE_DEG 30.0f
#define ATT_MAX_YAW_RATE_DPS 90.0f
