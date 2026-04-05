#pragma once
#include "config.h"

class Button {
  public:
    Button(int pin);
    bool isPressed();
    bool isReleased();
    bool heldFor(uint16_t duration_ms);

  private:
    uint8_t _pin;
    bool _lastState;
    uint64_t _lastEdgeTime;
    uint64_t _holdStartTime;
};