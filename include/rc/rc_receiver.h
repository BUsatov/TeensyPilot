#pragma once

#include <stdint.h>

struct RcChannels {
  static constexpr uint8_t kMaxChannels = 16;
  uint16_t ch[kMaxChannels];
  uint32_t timestamp_us;
  static constexpr uint8_t kRoll = 0;
  static constexpr uint8_t kPitch = 1;
  static constexpr uint8_t kThrottle = 2;
  static constexpr uint8_t kYaw = 3;
};

class RcReceiver {
public:
  virtual ~RcReceiver() = default;

  virtual bool begin() = 0;
  virtual bool read(RcChannels &out) = 0;
};
