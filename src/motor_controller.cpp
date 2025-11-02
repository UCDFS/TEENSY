/**
 * @file motor_controller.cpp
 * @brief Manages Bamocar D3 inverter communications and torque control over CAN.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#include "motor_controller.h"
#include "header.h"
#include <FlexCAN_T4.h>
#include "logging.h"


// ---------- CAN ----------
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
static constexpr uint32_t BAMOCAR_TX_ID = 0x181; // inverter -> us
static constexpr uint32_t BAMOCAR_RX_ID = 0x201; // us -> inverter

// ---------- State ----------
static bool readyToDrive = false;
static bool faultActive  = false;
static bool rtdRequestPending = false;
static uint32_t tPendingReady = 0;
static uint16_t statusWord = 0;
static int rpmFeedback = 0;
static float dcBusVoltage = 0.0f;
static int16_t lastTorque = 0;

// timers
static uint32_t tLastReissue = 0;
static uint32_t tBuzzerOff   = 0;

// ---------- helpers ----------
static inline void send3(uint8_t b0, uint8_t b1, uint8_t b2) {
  CAN_message_t m{};
  m.id = BAMOCAR_RX_ID;
  m.len = 3;
  m.buf[0] = b0;
  m.buf[1] = b1;
  m.buf[2] = b2;
  Can1.write(m);

  // Logging::logCANFrame(m, "TX");
}

static inline void requestCyclic(uint8_t reg, uint8_t periodMs) { send3(0x3D, reg, periodMs); }
static inline void setCanTimeout(uint16_t ms) { send3(0xD0, ms & 0xFF, (ms >> 8) & 0xFF); }
static inline void clearErrors() { send3(0x8E, 0x00, 0x00); }
static inline void lockDrive()   { send3(0x51, 0x04, 0x00); }
static inline void enableDrive() { send3(0x51, 0x00, 0x00); }
static inline void setTorqueRaw(int16_t tq) { send3(0x90, tq & 0xFF, (tq >> 8) & 0xFF); }

static void handleStatusWord(uint16_t word) {
  statusWord = word;
  faultActive = (word & 0x0040) != 0;
  const bool enableBit = (word & 0x0001) != 0;
  const bool readyBit  = (word & 0x0004) != 0;

  if (faultActive) {
    if (readyToDrive && MotorController::DEBUG_MODE)
      Serial.println("Inverter fault. RTD cleared.");
    readyToDrive = false;
    if (rtdRequestPending && MotorController::DEBUG_MODE)
      Serial.println("Pending RTD cancelled due to fault.");
    rtdRequestPending = false;
    tPendingReady = 0;
    if (tBuzzerOff) {
      if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);
      tBuzzerOff = 0;
    }
    return;
  }

  if (enableBit && readyBit) {
    if (!readyToDrive) {
      readyToDrive = true;
      rtdRequestPending = false;
      tPendingReady = 0;
      if (BUZZER_PIN >= 0) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, HIGH);
        tBuzzerOff = millis() + 3000;
      }
  if (MotorController::DEBUG_MODE) Serial.println("Inverter confirmed RTD.");
    }
  } else if (readyToDrive) {
    readyToDrive = false;
    tPendingReady = 0;
    if (tBuzzerOff) {
      if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);
      tBuzzerOff = 0;
    }
  if (MotorController::DEBUG_MODE) Serial.println("Inverter exited RTD state.");
  }
}

static void readCAN() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Logging::logCANFrame(msg, "RX");
    if (msg.id != BAMOCAR_TX_ID || msg.len < 3) continue;
    const uint8_t reg = msg.buf[0];

    if (reg == 0x40) { // status word
      const uint16_t word = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      handleStatusWord(word);
    } else if (reg == 0x30) { // rpm
      rpmFeedback = int16_t(uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8));
    } else if (reg == 0xEB) { // dc bus voltage 0.1 V/LSB
      dcBusVoltage = 0.1f * float(uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8));
    }
  }
}

static bool brakeActive() {
  // Brake light output serves as the activation signal; HIGH implies pedal pressed
  pinMode(BrakeLight::BRAKE_LIGHT_PIN, INPUT); // ensure not driving it here
  return digitalRead(BrakeLight::BRAKE_LIGHT_PIN) == HIGH;
}

static bool rtdButtonPressed() {
  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);
  return digitalRead(RTD_BUTTON_PIN) == LOW; // active low
}

static void tryEnterRTD() {
  static bool holding = false;
  static uint32_t tStart = 0;

  if (readyToDrive || faultActive || rtdRequestPending) {
    holding = false;
    return;
  }

  if (!brakeActive() || !rtdButtonPressed()) {
    holding = false;
    return;
  }

  if (!holding) {
    holding = true;
    tStart = millis();
  }

  if (millis() - tStart < 1000) return;

  clearErrors();
  lockDrive();
  delay(50);
  enableDrive();

  rtdRequestPending = true;
  tPendingReady = millis();
  if (MotorController::DEBUG_MODE) Serial.println("RTD enable request sent.");

  holding = false;
}

// ---------- public API ----------
namespace MotorController {

void init() {
  if (MotorController::DEBUG_MODE) Serial.println("Motor Controller init");

  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);
  if (BUZZER_PIN >= 0) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // CAN
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD_RATE);
  delay(50);
  Serial.println("CAN bus initialized");

  clearErrors();
  setCanTimeout(CAN_TIMEOUT_MS);
  delay(50);
  Serial.println("Cleared errors and set CAN timeout");

  // set cyclic requests
  requestCyclic(0x40, STATUS_REQ_INTERVAL_MS); // status
  requestCyclic(0x30, RPM_REQ_INTERVAL_MS);    // rpm
  // one shot dc bus
  send3(0x3D, 0xEB, 0x00);

  tLastReissue = millis();
}

void update() {
  Can1.events();
  readCAN();

  if (rtdRequestPending && (millis() - tPendingReady) > 2000) {
    rtdRequestPending = false;
  if (MotorController::DEBUG_MODE) Serial.println("RTD enable request timed out.");
  }

  // re-issue cyclic requests every few seconds for keep-alive robustness
  if (millis() - tLastReissue > 5000) {
    requestCyclic(0x40, STATUS_REQ_INTERVAL_MS);
    requestCyclic(0x30, RPM_REQ_INTERVAL_MS);
    tLastReissue = millis();
  }

  // buzzer off timer
  if (tBuzzerOff && millis() >= tBuzzerOff) {
    if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);
    tBuzzerOff = 0;
  }

  // manage RTD entry
  if (!readyToDrive) {
    tryEnterRTD();
  }
}

void setTorque(int16_t torqueCounts) {
  if (!readyToDrive || faultActive) return;
  if (torqueCounts != lastTorque) {
    setTorqueRaw(torqueCounts);
    lastTorque = torqueCounts;
  }
}

bool ready() { return readyToDrive; }
bool faulted() { return faultActive; }
int rpm() { return rpmFeedback; }
float dcBus() { return dcBusVoltage; }

} // namespace MotorController
