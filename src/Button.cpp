#include "Button.h"

Button::Button(int pin) : _pin(pin),
                          _lastState(LOW), 
                          _lastEdgeTime(0), 
                          _holdStartTime(0) {
  pinMode(_pin, INPUT_PULLDOWN);
}

bool Button::isPressed() {
  bool currentState = digitalRead(_pin);
  uint32_t currentTime = millis();

  if (_lastState == LOW && currentState == HIGH 
      && (currentTime - _lastEdgeTime) > DEBOUNCE_MS) {
    _lastEdgeTime = currentTime;
    _lastState = currentState;
    return true;
  }

  _lastState = currentState;
  return false;
}

bool Button::isReleased() {
  bool currentState = digitalRead(_pin);
  uint32_t currentTime = millis();

  if (_lastState == HIGH && currentState == LOW 
      && (currentTime - _lastEdgeTime) > DEBOUNCE_MS) {
    _lastEdgeTime = currentTime;
    _lastState = currentState;
    return true;
  }

  _lastState = currentState;
  return false;
}

bool Button::heldFor(uint16_t duration_ms) {
  if (digitalRead(_pin) == LOW) {
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

