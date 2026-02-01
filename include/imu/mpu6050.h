#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "imu/imu.h"

class Mpu6050Driver final : public Imu {
public:
  explicit Mpu6050Driver(uint8_t address = 0x68);

  bool begin() override;
  bool calibrate(uint16_t samples, uint16_t delay_ms) override;
  bool read(ImuSample &sample) override;

private:
  static constexpr uint8_t kRegPwrMgmt1 = 0x6B;
  static constexpr uint8_t kRegSmplrtDiv = 0x19;
  static constexpr uint8_t kRegConfig = 0x1A;
  static constexpr uint8_t kRegGyroConfig = 0x1B;
  static constexpr uint8_t kRegAccelConfig = 0x1C;
  static constexpr uint8_t kRegAccelXoutH = 0x3B;

  static constexpr float kAccLsbPerG = 4096.0f;
  static constexpr float kGyroLsbPerDps = 65.5f;

  uint8_t addr_;
  int16_t ax_bias_;
  int16_t ay_bias_;
  int16_t az_bias_;
  int16_t gx_bias_;
  int16_t gy_bias_;
  int16_t gz_bias_;

  void writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t *buf, size_t len);
  bool readRaw(ImuSample &sample);
  void applyCalibration(ImuSample &sample) const;
  void convert(ImuSample &sample) const;
};
