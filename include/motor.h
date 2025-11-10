
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

  void transmit(int function, int val, int val2 = 0x00);
  void set(int function, int val);
  void request(int field);

  namespace ADDRS {
  static constexpr int CANCEL_ERRORS = 0x8E; // Cannot confirm in docs
  static constexpr int TIMEOUT = 0xD0;       // pg 157
  static constexpr int SETPOINT = 0x90;      // pg 80
  static constexpr int MODE_BIT = 0x51;      // pg 29
  static constexpr int READ = 0x3D;          // pg 59
  } // namespace ADDRS

  String generateFilename();

  static constexpr int USB_DEBUGGING_PORT = 115200;
  static constexpr int ESP8266_PORT = 115200;
  static constexpr int BAUDRATE = 500000;
  static constexpr int ANALOG_RESOLUTION = 12;
  static constexpr int CAN_TIMEOUT = 2000;
};

#endif // INCLUDE_INCLUDE_MOTOR_H_
