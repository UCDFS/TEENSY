#include <cstdint>
#include <optional>

#ifndef INCLUDE_INCLUDE_MOTOR_H_
#define INCLUDE_INCLUDE_MOTOR_H_
/// This class should be used by the user directly

#include <FlexCAN_T4.h>
class Motor {
public:
  enum class MotorResponse {
    OK,
    NEGATIVE_TORQUE,
    OVERTORQUE,
    CAN_ERROR,
    DISABLED
  };
  ///@argument brake_active A pointer to a variable representing if the brake
  /// lights should be on. Should be upgraded periodicaly
  Motor(bool *brake_active);
  Motor(Motor &&) = default;
  Motor(const Motor &) = default;
  Motor &operator=(Motor &&) = default;
  Motor &operator=(const Motor &) = default;
  ~Motor();
  MotorResponse setTorque(double desired_Nm);

  inline void setCanTimeout(uint16_t ms);
  inline void clearErrors();
  inline void lockDrive();
  inline void enableDrive();
  void update();

  inline bool ready() { return readyToDrive; }
  inline bool faulted() { return faultActive; }
  inline int rpm() { return rpmFeedback; }
  inline float dcBus() { return dcBusVoltage; }

  // ---------- State ----------
private:
  inline void setTorqueRaw(int16_t tq);
  static bool readyToDrive;
  static bool faultActive;
  static bool rtdRequestPending;
  static uint32_t tPendingReady;
  static uint16_t statusWord;
  static int rpmFeedback;
  static float dcBusVoltage;
  static double lastTorque;

  // timers
  static uint32_t tLastReissue;
  static uint32_t tBuzzerOff;

  inline void requestCyclic(uint8_t reg, uint8_t periodMs) {
    send3(ADDRS::READ, reg, periodMs);
  }
  static constexpr int BUZZER_PIN = 0; // CHANGEME

  // CHANGEME
  static constexpr double MAX_TORQUE = 0.0;

  void readCAN();
  void transmit(int function, int val, int val2 = 0x00);

  inline void send3(uint8_t b0, uint8_t b1, uint8_t b2);
  void set(int function, int val);
  // uint16_t request_once(int field);
  void tryEnterRTD();
  bool getStatusField(uint8_t field);
  bool getError(uint8_t field);
  bool getWarning(uint8_t field);

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
    static constexpr uint8_t SPECIAL_SETTINGS = 0xDC;
    static constexpr uint8_t LOCK_DRIVE_BIT = 21; // pg 151
    static constexpr uint8_t WARNING_ERROR = 0x8F;

    static constexpr uint8_t STATUS_ENABLED = 0;
    static constexpr uint8_t STATUS_OK = 4;
    static constexpr uint8_t STATUS_STANDSTILL = 9;
    static constexpr uint8_t ERROR_CAN_TIMEOUT = 3;
    static constexpr uint8_t ERROR_MOTOR_OVERHEAT = 6;
    static constexpr uint8_t ERROR_ENDSTAGE_OVERHEAT = 7;

  }; // struct ADDRS

  String generateFilename();

  static constexpr int USB_DEBUGGING_PORT = 115200;
  static constexpr int ESP8266_PORT = 115200;
  static constexpr int BAUDRATE = 500000;
  static constexpr int ANALOG_RESOLUTION = 12;
  static constexpr int CAN_TIMEOUT = 2000;
  static constexpr int RTD_BUTTON_PIN = 0;         // CHANGEME
  static constexpr int STATUS_REQ_INTERVAL_MS = 0; // CHANGEME
  static constexpr int RPM_REQ_INTERVAL_MS = 0;    // CHANGEME

  // NOTE: These were orignialy static, but since there should only ever be a
  // single instance of Motor, this shouldn't be an issue.
  // ---------- CAN ----------
  static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
  static constexpr uint32_t BAMOCAR_TX_ID = 0x181; // inverter -> us
  static constexpr uint32_t BAMOCAR_RX_ID = 0x201; // us -> inverter

  void handleStatusWord(uint16_t word);
  static constexpr bool DEBUG_MODE = true;

  bool rtdButtonPressed();

  std::optional<bool> getFromBitfield(uint16_t bitfieldAddr, uint8_t bit);

  bool *brake_active;
};

#endif // INCLUDE_INCLUDE_MOTOR_H_
