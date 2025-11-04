#ifndef INCLUDE_INCLUDE_MOTOR_HAL_H_
#define INCLUDE_INCLUDE_MOTOR_HAL_H_
/// The motor API is split into two parts, a HAL that sends the CAN data
/// directly, and an API that the user should use The API makes the HAL cleaner
/// to use and makes error handling easier

#include <FlexCAN_T4.h>
#include <SD.h>
#include <SPI.h>

class MotorHAL {
public:
  MotorHAL();
  MotorHAL(MotorHAL &&) = default;
  MotorHAL(const MotorHAL &) = default;
  MotorHAL &operator=(MotorHAL &&) = default;
  MotorHAL &operator=(const MotorHAL &) = default;
  ~MotorHAL();
// ---------- BAMOCAR IDs ----------
#define BAMOCAR_RX_ID 0x201 // Teensy → Bamocar
#define BAMOCAR_TX_ID 0x181 // Bamocar → Teensy

// ---------- User constants ----------
#define MAX_ACCEL_PERCENT 50
#define TORQUE_MAX 32767
  const int chipSelect = BUILTIN_SDCARD;
  // ---------- CAN setup ----------
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

  // ---------- Globals ----------
  File logFile;
  int currentStep = 0;
  int16_t currentTorque = 0;
  uint32_t lastTorqueSend = 0;
  bool bamocarOnline = false;
  int rpmFeedback = 0;
  int statusWord = 0;
  float dcBusVoltage = 0.0;

  // ---------- Function prototypes ----------
  void executeStep(int step);
  void readCanMessages();
  void sendCAN(const CAN_message_t &msg);
  void requestStatusCyclic(uint8_t interval_ms);
  void requestStatusOnce();
  void requestSpeedCyclic(uint8_t interval_ms);
  void requestDCBusOnce();
  void clearErrors();
  void enableDrive();
  void disableDrive();
  void sendTorqueCommand(int16_t torqueValue);
  void configureCanTimeout(uint16_t ms);
  void updateTorqueFromPot();
  String interpretBamocarMessage(const CAN_message_t &msg);
  void logCANFrame(const CAN_message_t &msg, const char *dir);
  String generateFilename();
  void dumpLogToSerial();
  void sendTelemetry(); // new
  void setup();
  void loop();

private:
};

MotorHAL::MotorHAL() {}

MotorHAL::~MotorHAL() {}

#endif // INCLUDE_INCLUDE_MOTOR_HAL_H_
