
#include "units.h"
#ifndef INCLUDE_INCLUDE_MOTOR_H_
#define INCLUDE_INCLUDE_MOTOR_H_
/// This class should be used by the user directly

#include "motor_hal.h"
class Motor : private MotorHAL {
public:
  enum class MotorResponse { OK, NEGATIVE_TORQUE, OVERTORQUE, CAN_ERROR };
  Motor();
  Motor(Motor &&) = default;
  Motor(const Motor &) = default;
  Motor &operator=(Motor &&) = default;
  Motor &operator=(const Motor &) = default;
  ~Motor();
  MotorResponse set_torque(units::torque::newton_meter_t desired);

private:
  // CHANGEME
  static constexpr units::torque::newton_meter_t MAX_TORQUE =
      units::torque::newton_meter_t{0};
};

#endif // INCLUDE_INCLUDE_MOTOR_H_
