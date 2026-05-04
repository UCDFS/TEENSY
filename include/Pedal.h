#pragma once
#include "config.h"

class Pedal {
private:
  float appsPercent(int raw, int rest, int full);
  bool fault = false;
  uint8_t plausibilityCount = 0;
public:
  Pedal();
  int16_t apps1Raw = 0;
  int16_t apps2Raw = 0;
  int16_t read();
};