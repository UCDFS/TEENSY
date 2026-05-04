#pragma once
#include "config.h"

class Button {
private:
  uint8_t _pin;
  bool _lastState;
  uint64_t _lastEdgeTime;
  uint64_t _holdStartTime;
public:
  Button(int pin);
  bool isPressed();
  bool isReleased();
  bool heldFor(uint16_t duration_ms);
};