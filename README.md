# TeensyPilot

Autopilot prototype for Teensy 4.0 with MPU‑6050 IMU, CRSF RC input, PWM ESC output, and multi‑rate control loop.

## Project Layout

```
include/
  control/   # controllers, gains, modes, config
  imu/       # IMU drivers + filters
  rc/        # RC channel interface
  hal/       # PWM/ESC, mixer, logging
  state/     # state estimators
src/
  control/
  imu/
  rc/
  hal/
  state/
  main.cpp
```

## Hardware

- **Teensy 4.0**
- **MPU‑6050** on I2C (SDA 18, SCL 19)
- **CRSF receiver** on Serial2 (pin 14)
- **4x ESC** on pins 1, 2, 3, 4

Example hardware setup reference:
```
https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone
```

## Build & Upload (PlatformIO)

`platformio.ini` targets Teensy 4.0.

1) Build: `pio run`
2) Upload: `pio run -t upload`
3) Serial monitor: `pio device monitor -b 115200`

## RC + Modes

- **Arm**: RC9 (channel index 8), armed when >= 1500
- **Modes** (RC10 / channel index 9):
  - `< 900` → Stabilize
  - `900–1499` → Constant
  - `>= 1500` → Acro

## Control Loop

- **IMU task**: 1 kHz
- **Control task**: 250 Hz
- **Log task**: 50 Hz

Scheduler lives in `src/main.cpp`.

## IMU Filters

Config in `include/imu/imu_filters.h`.

- Set LPF cutoffs to `0.0f` to disable
- Notch filters (up to 3 bands) are optional
- Notch center can scale with throttle

## Logging

CSV is printed over Serial and includes:
- IMU (accel/gyro)
- Attitude
- RC channels
- Mode
- Motor outputs
- Controller debug (setpoints, rates, PID outputs)

Notebook `log_plot.ipynb` is included for plotting.

## Files to Start With

- `src/main.cpp` — app wiring + scheduler
- `src/control/stabilize_mode.cpp` — stabilize controller logic
- `include/control/control_gains.h` — PID gains
- `include/imu/imu_filters.h` — filter settings

## Notes

- State estimator currently uses complementary or Madgwick (default complementary).
- Yaw drifts without magnetometer.
