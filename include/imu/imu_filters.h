#pragma once

#include <stdint.h>

// Low-pass filter config (Hz). Set to 0 to disable.
#define IMU_LPF_GYRO_HZ 100.0f
#define IMU_LPF_ACCEL_HZ 40.0f

// Throttle range used for notch scaling.
#define IMU_NOTCH_THROTTLE_MIN_US 1000.0f
#define IMU_NOTCH_THROTTLE_MAX_US 2000.0f

// Notch bands (set base Hz or width to 0 to disable band).
// Band 1
#define IMU_NOTCH1_GYRO_HZ 85.0f
#define IMU_NOTCH1_GYRO_Q  8.0f
#define IMU_NOTCH1_ACCEL_HZ 0.0f
#define IMU_NOTCH1_ACCEL_Q  0.0f
// Band 2
#define IMU_NOTCH2_GYRO_HZ 22.0f
#define IMU_NOTCH2_GYRO_Q  6.0f
#define IMU_NOTCH2_ACCEL_HZ 0.0f
#define IMU_NOTCH2_ACCEL_Q  0.0f
// Band 3
#define IMU_NOTCH3_GYRO_HZ 0.0f
#define IMU_NOTCH3_GYRO_Q  0.0f
#define IMU_NOTCH3_ACCEL_HZ 0.0f
#define IMU_NOTCH3_ACCEL_Q  0.0f

// Throttle-dependent scaling: center_hz = base_hz * (1 + scale * throttle_norm)
// Set scale to 0 to disable throttle dependence.
#define IMU_NOTCH_GYRO_SCALE 0.3f
#define IMU_NOTCH_ACCEL_SCALE 0.0f
