#include "imu/mpu6050.h"

Mpu6050Driver::Mpu6050Driver(uint8_t address)
    : addr_(address),
      ax_bias_(0),
      ay_bias_(0),
      az_bias_(0),
      gx_bias_(0),
      gy_bias_(0),
      gz_bias_(0) {}

void Mpu6050Driver::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool Mpu6050Driver::readReg(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  size_t got = Wire.requestFrom(addr_, static_cast<uint8_t>(len));
  if (got != len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool Mpu6050Driver::begin() {
  ax_bias_ = 0;
  ay_bias_ = 0;
  az_bias_ = 0;
  gx_bias_ = 0;
  gy_bias_ = 0;
  gz_bias_ = 0;

  writeReg(kRegPwrMgmt1, 0x00);
  delay(100);

  writeReg(kRegConfig, 0x05);        // DLPF_CFG=3 (~44 Hz gyro/accel)
  writeReg(kRegSmplrtDiv, 0x03);     // 1 kHz / (1 + 3) = 250 Hz
  writeReg(kRegGyroConfig, 0x8);     // ±250 dps
  writeReg(kRegAccelConfig, 0x43);   // ±2 g

  return true;
}

bool Mpu6050Driver::readRaw(ImuSample &sample) {
  uint8_t buf[14];
  if (!readReg(kRegAccelXoutH, buf, sizeof(buf))) {
    return false;
  }

  sample.ax_raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  sample.ay_raw = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  sample.az_raw = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  sample.gx_raw = static_cast<int16_t>((buf[8] << 8) | buf[9]);
  sample.gy_raw = static_cast<int16_t>((buf[10] << 8) | buf[11]);
  sample.gz_raw = static_cast<int16_t>((buf[12] << 8) | buf[13]);

  return true;
}

void Mpu6050Driver::applyCalibration(ImuSample &sample) const {
  sample.ax_raw -= ax_bias_;
  sample.ay_raw -= ay_bias_;
  sample.az_raw -= az_bias_;
  sample.gx_raw -= gx_bias_;
  sample.gy_raw -= gy_bias_;
  sample.gz_raw -= gz_bias_;
}

void Mpu6050Driver::convert(ImuSample &sample) const {
  sample.ax_g = sample.ax_raw / kAccLsbPerG;
  sample.ay_g = sample.ay_raw / kAccLsbPerG;
  sample.az_g = sample.az_raw / kAccLsbPerG;
  sample.gx_dps = sample.gx_raw / kGyroLsbPerDps;
  sample.gy_dps = sample.gy_raw / kGyroLsbPerDps;
  sample.gz_dps = sample.gz_raw / kGyroLsbPerDps;
}

bool Mpu6050Driver::read(ImuSample &sample) {
  if (!readRaw(sample)) {
    return false;
  }
  applyCalibration(sample);
  convert(sample);
  return true;
}

bool Mpu6050Driver::calibrate(uint16_t samples, uint16_t delay_ms) {
  int32_t ax_sum = 0;
  int32_t ay_sum = 0;
  int32_t az_sum = 0;
  int32_t gx_sum = 0;
  int32_t gy_sum = 0;
  int32_t gz_sum = 0;

  for (uint16_t i = 0; i < samples; i++) {
    ImuSample s{};
    if (!readRaw(s)) {
      return false;
    }
    ax_sum += s.ax_raw;
    ay_sum += s.ay_raw;
    az_sum += s.az_raw;
    gx_sum += s.gx_raw;
    gy_sum += s.gy_raw;
    gz_sum += s.gz_raw;
    delay(delay_ms);
  }

  int32_t ax_avg = ax_sum / static_cast<int32_t>(samples);
  int32_t ay_avg = ay_sum / static_cast<int32_t>(samples);
  int32_t az_avg = az_sum / static_cast<int32_t>(samples);
  int32_t gx_avg = gx_sum / static_cast<int32_t>(samples);
  int32_t gy_avg = gy_sum / static_cast<int32_t>(samples);
  int32_t gz_avg = gz_sum / static_cast<int32_t>(samples);

  ax_bias_ = static_cast<int16_t>(ax_avg);
  ay_bias_ = static_cast<int16_t>(ay_avg);
  az_bias_ = static_cast<int16_t>(az_avg - static_cast<int32_t>(kAccLsbPerG));
  gx_bias_ = static_cast<int16_t>(gx_avg);
  gy_bias_ = static_cast<int16_t>(gy_avg);
  gz_bias_ = static_cast<int16_t>(gz_avg);

  return true;
}
