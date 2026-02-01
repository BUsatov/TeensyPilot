#include <Arduino.h>
#include <Wire.h>

#include "imu/imu.h"
#include "imu/mpu6050.h"
#include "state/state_estimator.h"
#include "state/comp_estimator.h"
#include "state/madgwick_estimator.h"
#include "hal/logger.h"
#include "hal/serial_logger.h"
#include "rc/rc_receiver.h"
#include "CRSFforArduino.hpp"
#include "hal/esc_pwm.h"
#include "hal/motor_mixer.h"
#include "control/control_state.h"
#include "control/control_config.h"
#include "control/flight_mode.h"
#include "control/constant_mode.h"
#include "control/stabilize_mode.h"
#include "control/acro_mode.h"
#include "control/flight_mode_factory.h"
#include "imu/imu_filtering.h"

namespace {
struct App {
  static constexpr uint8_t kMotorPins[EscPwm::kMotorCount] = {2, 3, 4, 1};
  static constexpr uint32_t kImuPeriodUs = 4000;    // 250 Hz
  static constexpr uint32_t kControlPeriodUs = 4000; // 250 Hz
  static constexpr uint32_t kLogPeriodUs = 4000;   // 250 Hz

  Mpu6050Driver imu{0x68};
  ComplementaryEstimator estimator{0.8f};
  MadgwickEstimator madgwick{0.1f};
  StateEstimator *state_estimator = &madgwick; // default: complementary
  SerialLogger logger{Serial};
  uint32_t last_us = 0;
  CRSFforArduino crsf{&Serial2};
  bool crsf_ok = false;
  EscPwm esc{kMotorPins};
  MotorMixer mixer{esc};

  RcChannels last_ch{};
  uint32_t last_rc_update_us = 0;
  ImuSample last_sample{};
  bool last_imu_ok = false;
  StateEstimate last_est{};
  float last_dt = 0.0f;
  ImuFilterBank imu_filters;

  ControlState control_state{ArmState::kDisarmed, FlightMode::kConstant};
  ConstantMode constant_mode{mixer};
  StabilizeMode stabilize_mode{mixer};
  AcroMode acro_mode{mixer};
  FlightModeController *flight_mode = &constant_mode;

  static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData);
  void setup();
  void loop();
  void imuTask(uint32_t now_us);
  void controlTask(uint32_t now_us);
  void logTask(uint32_t now_us);

  struct Task {
    uint32_t period_us;
    uint32_t last_us;
    void (App::*fn)(uint32_t now_us);
  };

  Task tasks[3] = {
      {kImuPeriodUs, 0, &App::imuTask},
      {kControlPeriodUs, 0, &App::controlTask},
      {kLogPeriodUs, 0, &App::logTask},
  };
};

App app;
}  // namespace

void App::onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData) {
  if (rcData == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < RcChannels::kMaxChannels; i++) {
    app.last_ch.ch[i] = app.crsf.rcToUs(rcData->value[i]);
  }
  app.last_rc_update_us = micros();
}

void App::setup() {
  Serial.begin(115200);
  logger.begin();
  while (!Serial && millis() < 2000) {
    // Wait for Serial on USB, but don't block forever.
  }

  Wire.begin();        // Teensy 4.0: SDA=18, SCL=19
  Wire.setClock(400000);

  crsf_ok = crsf.begin();
  if (crsf_ok) {
    crsf.setRcChannelsCallback(onReceiveRcChannels);
  } else {
    crsf.end();
  }
  esc.begin(1000, 2000);
  mixer.setThrottleUs(kStopPwmUs);

  if (!imu.begin()) {
    Serial.println("MPU-6050 init failed");
  } else {
    Serial.println("MPU-6050 init OK");
  }

  Serial.println("Calibrating... keep sensor still");
  if (!imu.calibrate(500, 2)) {
    Serial.println("MPU-6050 calibration failed");
  } else {
    Serial.println("MPU-6050 calibration OK");
  }

  logger.logLine("t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,throttle_us,motor_us,m1,m2,m3,m4,mode,rc_roll,rc_pitch,rc_throttle,rc_yaw,rc_age_us,att_sp_roll,att_sp_pitch,att_sp_yaw,rate_sp_roll,rate_sp_pitch,rate_sp_yaw,rate_meas_roll,rate_meas_pitch,rate_meas_yaw,rate_out_roll,rate_out_pitch,rate_out_yaw");

  estimator.reset();
  madgwick.reset();
  last_us = micros();
}

void App::loop() {
  uint32_t now_us = micros();
  for (auto &task : tasks) {
    if (task.last_us == 0) {
      task.last_us = now_us;
    }
    if (static_cast<int32_t>(now_us - task.last_us) >= static_cast<int32_t>(task.period_us)) {
      task.last_us += task.period_us;
      (this->*task.fn)(now_us);
    }
  }
}

void App::imuTask(uint32_t now_us) {
  if (crsf_ok) {
    crsf.update();
  }

  last_imu_ok = imu.read(last_sample);
  if (last_imu_ok) {
    float dt = (now_us - last_us) / 1000000.0f;
    last_us = now_us;
    imu_filters.updateThrottleUs(last_ch.ch[RcChannels::kThrottle]);
    imu_filters.apply(last_sample, dt);
    state_estimator->update(last_sample.gx_dps, last_sample.gy_dps, last_sample.gz_dps,
                            last_sample.ax_g, last_sample.ay_g, last_sample.az_g, dt);
    last_est = state_estimator->state();
    last_est.roll_rate_dps = last_sample.gx_dps;
    last_est.pitch_rate_dps = last_sample.gy_dps;
    last_est.yaw_rate_dps = last_sample.gz_dps;
    last_dt = dt;
  }
}

void App::controlTask(uint32_t now_us) {
  (void)now_us;
  updateControlStateFromRc(control_state, last_ch);
  flight_mode = &selectModeController(control_state.mode,
                                      stabilize_mode,
                                      constant_mode,
                                      acro_mode);

  if (control_state.arm == ArmState::kDisarmed) {
    mixer.setThrottleUs(kStopPwmUs);
    return;
  }

  flight_mode->apply(last_ch, last_est, last_dt);
}

void App::logTask(uint32_t now_us) {
  if (!last_imu_ok) {
    return;
  }
  char line[300];
  uint16_t motor_out[EscPwm::kMotorCount] = {0, 0, 0, 0};
  mixer.getLastOutputs(motor_out);
  uint32_t t_us = now_us;
  uint32_t rc_age_us = (last_rc_update_us == 0) ? 0 : (t_us - last_rc_update_us);
  StabilizeMode::Debug dbg{};
  if (control_state.mode == FlightMode::kStabilize) {
    dbg = stabilize_mode.debug();
  }
  snprintf(line, sizeof(line),
           "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           static_cast<unsigned long>(t_us),
           last_sample.ax_g, last_sample.ay_g, last_sample.az_g,
           last_sample.gx_dps, last_sample.gy_dps, last_sample.gz_dps,
           last_est.roll_deg, last_est.pitch_deg,
           static_cast<unsigned int>(flight_mode->lastThrottleUs()),
           static_cast<unsigned int>(flight_mode->lastMotorUs()),
           static_cast<unsigned int>(motor_out[0]),
           static_cast<unsigned int>(motor_out[1]),
           static_cast<unsigned int>(motor_out[2]),
           static_cast<unsigned int>(motor_out[3]),
           static_cast<unsigned int>(control_state.mode),
           static_cast<unsigned int>(last_ch.ch[RcChannels::kRoll]),
           static_cast<unsigned int>(last_ch.ch[RcChannels::kPitch]),
           static_cast<unsigned int>(last_ch.ch[RcChannels::kThrottle]),
           static_cast<unsigned int>(last_ch.ch[RcChannels::kYaw]),
           static_cast<unsigned long>(rc_age_us),
           dbg.att_sp_roll_deg, dbg.att_sp_pitch_deg, dbg.att_sp_yaw_deg,
           dbg.rate_sp_roll_dps, dbg.rate_sp_pitch_dps, dbg.rate_sp_yaw_dps,
           dbg.rate_meas_roll_dps, dbg.rate_meas_pitch_dps, dbg.rate_meas_yaw_dps,
           dbg.rate_out_roll_us, dbg.rate_out_pitch_us, dbg.rate_out_yaw_us);
  logger.logLine(line);
}

void setup() {
  app.setup();
}

void loop() {
  app.loop();
}
