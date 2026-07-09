#pragma once
#include "config.h"

// TE M7139-500PG-P00000 pressure transducer, through the breakout board's
// 10k/20k divider. Fit from bench calibration against a reference gauge.
static inline float adcToBar(uint16_t adc) {
  return 0.010415f * (float)adc - 4.309225f;
}

static inline float adcToPsi(uint16_t adc) {
  return 0.151062f * (float)adc - 62.5f;
}

class BrakePedal {
private:
  bool pressed = false;
public:
  int16_t frontRaw = 0;
  int16_t rearRaw = 0;
  bool read();
};
