
#include "units.h"
#ifndef INCLUDE_INCLUDE_MOTOR_H_
#define INCLUDE_INCLUDE_MOTOR_H_
/// This class should be used by the user directly

#include "motor_hal.h"
#include <FlexCAN_T4.h>
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
  static inline void requestCyclic(uint8_t reg, uint8_t periodMs) {
    send3(ADDRS::READ, reg, periodMs);
  }
  static inline void setCanTimeout(uint16_t ms);
  static inline void clearErrors();
  static inline void lockDrive();
  static inline void enableDrive();
  static inline void setTorqueRaw(int16_t tq);

private:
  // CHANGEME
  static constexpr units::torque::newton_meter_t MAX_TORQUE =
      units::torque::newton_meter_t{0};

  void transmit(int function, int val, int val2 = 0x00);

  static inline void send3(uint8_t b0, uint8_t b1, uint8_t b2);
  void set(int function, int val);
  void request_once(int field);

  struct ADDRS {
    static constexpr uint8_t CANCEL_ERRORS = 0x8E; // Cannot confirm in docs
    static constexpr uint8_t TIMEOUT = 0xD0;       // pg 157
    static constexpr uint8_t SETPOINT = 0x90;      // pg 80
    static constexpr uint8_t MODE_BIT = 0x51;      // pg 29
    static constexpr uint8_t READ = 0x3D;          // pg 59
    static constexpr uint8_t READ_DC = 0xEB;
    static constexpr uint8_t STATUS = 0x40; // pg 133
    static constexpr uint8_t RPM = 0x30;    // pg 142
    static constexpr uint8_t LOCK_DRIVE = 0x04;
  }; // struct ADDRS

  String generateFilename();

  static constexpr int USB_DEBUGGING_PORT = 115200;
  static constexpr int ESP8266_PORT = 115200;
  static constexpr int BAUDRATE = 500000;
  static constexpr int ANALOG_RESOLUTION = 12;
  static constexpr int CAN_TIMEOUT = 2000;

  // NOTE: These were orignialy static, but since there should only ever be a
  // single instance of Motor, this shouldn't be an issue.
  // ---------- CAN ----------
  static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
  constexpr uint32_t BAMOCAR_TX_ID = 0x181; // inverter -> us
  constexpr uint32_t BAMOCAR_RX_ID = 0x201; // us -> inverter

public:
  // ---------- State ----------
  static bool readyToDrive;
  static bool faultActive;
  static bool rtdRequestPending;
  static uint32_t tPendingReady;
  static uint16_t statusWord;
  static int rpmFeedback;
  static float dcBusVoltage;
  static int16_t lastTorque;

  // timers
  static uint32_t tLastReissue;
  static uint32_t tBuzzerOff;

private:
};

#endif // INCLUDE_INCLUDE_MOTOR_H_
