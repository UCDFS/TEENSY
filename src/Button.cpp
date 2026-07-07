#include "Button.h"

// Buttons wire the pin to GND (no pull-ups on the breakout board):
// INPUT_PULLUP, pin reads LOW while pressed. State variables hold the
// logical pressed state, not the raw pin level.
Button::Button(int pin) : _pin(pin),
                          _lastState(false),
                          _lastEdgeTime(0),
                          _holdStartTime(0) {
  pinMode(_pin, INPUT_PULLUP);
}

bool Button::isPressed() {
  bool currentState = digitalRead(_pin) == LOW;
  uint32_t currentTime = millis();

  if (!_lastState && currentState
      && (currentTime - _lastEdgeTime) > DEBOUNCE_MS) {
    _lastEdgeTime = currentTime;
    _lastState = currentState;
    return true;
  }

  _lastState = currentState;
  return false;
}

bool Button::isReleased() {
  bool currentState = digitalRead(_pin) == LOW;
  uint32_t currentTime = millis();

  if (_lastState && !currentState
      && (currentTime - _lastEdgeTime) > DEBOUNCE_MS) {
    _lastEdgeTime = currentTime;
    _lastState = currentState;
    return true;
  }

  _lastState = currentState;
  return false;
}

bool Button::heldFor(uint16_t duration_ms) {
  if (digitalRead(_pin) == HIGH) {
    _holdStartTime = 0;
    return false;
  }

  if (this->isPressed()) {
    _holdStartTime = millis();
    return false;
  }

  if ((millis() - _holdStartTime) >= duration_ms) {
    _holdStartTime = 0;
    return true;
  }

  return false;
}

