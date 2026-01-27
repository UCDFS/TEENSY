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
  /// lights should be on. Should be updated periodicaly
  Motor(bool *brake_active);
  Motor(Motor &&) = default;
  Motor(const Motor &) = default;
  Motor &operator=(Motor &&) = default;
  Motor &operator=(const Motor &) = default;
  ~Motor() = default;
  MotorResponse setTorque(double desired_Nm);

  inline void setCanTimeout(uint16_t ms);
  inline void clearErrors();
  inline void lockDrive();
  // Very similar to tryEnterRTD, but attempts to unlock the drive system as
  // well
  inline void enableDrive();
  void update();

  inline bool ready() { return readyToDrive; }
  inline bool faulted() { return faultActive; }
  inline int rpm() { return rpmFeedback; }
  inline float dcBus() { return dcBusVoltage; }

  // ---------- State ----------
private:
  //---
  static inline bool readyToDrive = false;
  static inline bool faultActive = false;
  static inline bool rtdRequestPending = false;
  static inline uint32_t tPendingReady = 0;
  static inline uint16_t statusWord = 0;
  static inline int rpmFeedback = 0;
  static inline float dcBusVoltage = 0.0f;
  static inline int16_t lastTorque = 0;
  static inline uint32_t tLastReissue = 0;
  static inline uint32_t tBuzzerOff = 0;
  //---

  inline void requestCyclic(uint8_t reg, uint8_t periodMs) {
    send3(ADDRS::READ, reg, periodMs);
  }
  static constexpr int BUZZER_PIN = 0; // CHANGEME

  // CHANGEME
  static constexpr double MAX_TORQUE = 0.0;

  void transmit(int function, int val, int val2 = 0x00);

  inline void send3(uint8_t b0, uint8_t b1, uint8_t b2) {
    CAN_message_t m{};
    m.id = BAMOCAR_RX_ID;
    m.len = 3;
    m.buf[0] = b0;
    m.buf[1] = b1;
    m.buf[2] = b2;
    Can1.write(m);

    // Logging::logCANFrame(m, "TX");
  }
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

    // Errors occupy the 0x8F low bitmask, whereas warning occupy the 0x8F high
    // bitmask
    //
    // All avalible on page 148
    static constexpr uint8_t ERROR_EPROM_READ = 0;
    static constexpr uint8_t ERROR_HW_FAULT = 1;
    static constexpr uint8_t ERROR_RFE_INPUT_MISSING = 2;
    static constexpr uint8_t ERROR_CAN_TIMEOUT = 3;
    static constexpr uint8_t ERROR_FEEDBACK_SIGNAL = 4;
    static constexpr uint8_t ERROR_MAINS_VOLTAGE_MIN = 5;
    static constexpr uint8_t ERROR_MOTOR_OVERHEAT = 6;
    static constexpr uint8_t ERROR_ENDSTAGE_OVERHEAT = 7;
    static constexpr uint8_t ERROR_MAINS_VOLTAGE_MAX = 8;
    static constexpr uint8_t ERROR_CRITICAL_AC_CURRENT = 9;
    static constexpr uint8_t ERROR_RACE_AWAY = 10;
    static constexpr uint8_t ERROR_ECODE_TIMEOUT = 11;
    static constexpr uint8_t ERROR_WATCHDOG_RESET = 12;
    static constexpr uint8_t ERROR_AC_CURRENT_OFFSET = 13;
    static constexpr uint8_t ERROR_INTERNAL_HW_VOLTAGE = 14;
    static constexpr uint8_t ERROR_BLEED_RESISTOR_OVERLOAD = 15;

    static constexpr uint8_t WARNING_PARAMETER_CONFLICT = 16;
    static constexpr uint8_t WARNING_SPECIAL_CPU_FAULT = 17;
    static constexpr uint8_t WARNING_RFE_INPUT_MISSING = 18;
    static constexpr uint8_t WARNING_AUX_VOLTAGE_MIN = 19;
    static constexpr uint8_t WARNING_FEEDBACK_SIGNAL = 20;
    static constexpr uint8_t WARNING_5 = 21;
    static constexpr uint8_t WARNING_MOTOR_TEMP_LIMIT = 22;
    static constexpr uint8_t WARNING_IGBT_TEMP_LIMIT = 23;
    static constexpr uint8_t WARNING_VOUT_SATURATION_MAX = 24;
    static constexpr uint8_t WARNING_9 = 25;
    static constexpr uint8_t WARNING_SPEED_RESOLUTION_LIMIT = 26;
    static constexpr uint8_t WARNING_CHECK_ECODE_ID = 27;
    static constexpr uint8_t WARNING_TRIPZONE_GLITCH = 28;
    static constexpr uint8_t WARNING_ADC_SEQUENCER = 29;
    static constexpr uint8_t WARNING_ADC_MEASUREMENT = 30;
    static constexpr uint8_t WARNING_BLEEDER_RESISTOR_LOAD = 31;

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

  // ---------- CAN ----------
  static inline FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
  static constexpr uint32_t BAMOCAR_TX_ID = 0x181; // inverter -> us
  static constexpr uint32_t BAMOCAR_RX_ID = 0x201; // us -> inverter

  static constexpr bool DEBUG_MODE = true;

  bool rtdButtonPressed();

  // Currently risks blocking everything and starving the thread
  std::optional<bool> getFromBitfield(uint16_t bitfieldAddr, uint8_t bit);

  bool *brake_active;

  // Represents the state of a can.read() in a much more human-readable way
  struct FullCANFrame {
    uint16_t rawStatusWord;

    int16_t rpmFeedback;
    float dcBusVoltage;
    bool warning_present;

    struct ProcessedStatusWord {
      bool enableBit;
      bool readyBit;
      bool faultActive;
    } statusWord;
    struct Warnings {
      bool parameter_conflict;
      bool special_cpu_fault;
      bool rfe_input_missing;
      bool aux_voltage_min;
      bool feedback_signal;
      bool warn_5;
      bool motor_temp_limit;
      bool igbt_temp_limit;
      bool vout_saturation_max;
      bool warn_9;
      bool speed_resolution_limit;
      bool check_ecode_id;
      bool tripzone_glitch;
      bool adc_sequencer;
      bool adc_measurement;
      bool bleeder_resistor_load;
    } warnings;

    bool error_present;
    struct Errors {
      bool eprom_read;
      bool hw_fault;
      bool rfe_input_missing;
      bool can_timeout;
      bool feedback_signal;
      bool mains_voltage_min;
      bool motor_overheat;
      bool endstage_overheat;
      bool mains_voltage_max;
      bool critical_ac_current;
      bool race_away;
      bool ecode_timeout;
      bool watchdog_reset;
      bool ac_current_offset;
      bool internal_hw_voltage;
      bool bleed_resistor_overload;
    } errors;
  };

  FullCANFrame readCAN();

  FullCANFrame::ProcessedStatusWord handleStatusWord(uint16_t word);
};

#endif // INCLUDE_INCLUDE_MOTOR_H_
