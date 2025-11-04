#ifndef INCLUDE_INCLUDE_MOTOR_H_
#define INCLUDE_INCLUDE_MOTOR_H_
/// This class should be used by the user directly

#include "motor_hal.h"
class Motor : private MotorHAL {
public:
  Motor();
  Motor(Motor &&) = default;
  Motor(const Motor &) = default;
  Motor &operator=(Motor &&) = default;
  Motor &operator=(const Motor &) = default;
  ~Motor();

private:
};

Motor::Motor() {}

Motor::~Motor() {}

#endif // INCLUDE_INCLUDE_MOTOR_H_
