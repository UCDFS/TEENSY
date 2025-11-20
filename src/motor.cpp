#include "motor.h"
#include "WProgram.h"

// Static member definitions
/* FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Motor::Can1;
bool Motor::readyToDrive = false;
bool Motor::faultActive = false;
bool Motor::rtdRequestPending = false;
uint32_t Motor::tPendingReady = 0;
uint16_t Motor::statusWord = 0;
int Motor::rpmFeedback = 0;
float Motor::dcBusVoltage = 0.0f;
int16_t Motor::lastTorque = 0;
uint32_t Motor::tLastReissue = 0;
uint32_t Motor::tBuzzerOff = 0;
*/
/**
 * @file motor_controller.cpp
 * @brief Manages Bamocar D3 inverter communications and torque control over
 * CAN.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

// #include "brake_light.h"
// #include "header.h"
#include "motor.h"
#include <FlexCAN_T4.h>
#include <cstdint>
// ---------- helpers ----------
inline void Motor::send3(uint8_t b0, uint8_t b1, uint8_t b2) {
  CAN_message_t m{};
  m.id = BAMOCAR_RX_ID;
  m.len = 3;
  m.buf[0] = b0;
  m.buf[1] = b1;
  m.buf[2] = b2;
  Can1.write(m);

  // Logging::logCANFrame(m, "TX");
}

// TODO ask Shane about how this works and how "word" is derived and defined
void Motor::handleStatusWord(uint16_t word) {
  statusWord = word;
  faultActive = (word & 0x0040) != 0;
  const bool enableBit = (word & 0x0001) != 0;
  const bool readyBit = (word & 0x0004) != 0;

  if (faultActive) {
    if (readyToDrive && DEBUG_MODE)
      Serial.println("Inverter fault. RTD cleared.");
    readyToDrive = false;
    if (rtdRequestPending && DEBUG_MODE)
      Serial.println("Pending RTD cancelled due to fault.");
    rtdRequestPending = false;
    tPendingReady = 0;
    if (tBuzzerOff) {
      if (BUZZER_PIN >= 0)
        digitalWrite(BUZZER_PIN, LOW);
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
      if (DEBUG_MODE)
        Serial.println("Inverter confirmed RTD.");
    }
  } else if (readyToDrive) {
    readyToDrive = false;
    tPendingReady = 0;
    if (tBuzzerOff) {
      if (BUZZER_PIN >= 0)
        digitalWrite(BUZZER_PIN, LOW);
      tBuzzerOff = 0;
    }
    if (DEBUG_MODE)
      Serial.println("Inverter exited RTD state.");
  }
}

std::optional<bool> Motor::getFromBitfield(uint16_t bitfieldAddr, uint8_t bit) {

  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.id != BAMOCAR_TX_ID || msg.len < 3)
      continue;
    const uint8_t reg = msg.buf[0];
    if (reg == bitfieldAddr) {
      const uint16_t word = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      return std::optional<bool>{(word & bit) != 0};
    }
  }
  return std::nullopt;
}

void Motor::readCAN() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Logging::logCANFrame(msg, "RX");
    if (msg.id != BAMOCAR_TX_ID || msg.len < 3)
      continue;
    const uint8_t reg = msg.buf[0];

    if (reg == ADDRS::STATUS) { // status word
      const uint16_t word = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      this->handleStatusWord(word);
    } else if (reg == ADDRS::RPM) { // rpm
      rpmFeedback = int16_t(uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8));
    } else if (reg == ADDRS::READ_DC) { // dc bus voltage 0.1 V/LSB
      dcBusVoltage =
          0.1f * float(uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8));
    }
  }
}

// static bool brakeActive() { return BrakeLight::is_active(); }

bool Motor::rtdButtonPressed() {
  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);
  return digitalRead(RTD_BUTTON_PIN) == LOW; // active low
}

void Motor::tryEnterRTD() {
  static bool holding = false;
  static uint32_t tStart = 0;

  if (readyToDrive || faultActive || rtdRequestPending) {
    holding = false;
    return;
  }

  if (!brake_active || !rtdButtonPressed()) {
    holding = false;
    return;
  }

  if (!holding) {
    holding = true;
    tStart = millis();
  }

  if (millis() - tStart < 1000)
    return;

  clearErrors();
  lockDrive();
  delay(50);
  enableDrive();

  rtdRequestPending = true;
  tPendingReady = millis();
  if (DEBUG_MODE)
    Serial.println("RTD enable request sent.");

  holding = false;
}

// ---------- public API ----------

void init() {}

void Motor::update() {
  Can1.events();
  readCAN();

  if (rtdRequestPending && (millis() - tPendingReady) > 2000) {
    rtdRequestPending = false;
    if (DEBUG_MODE)
      Serial.println("RTD enable request timed out.");
  }

  // re-issue cyclic requests every few seconds for keep-alive robustness
  if (millis() - tLastReissue > 5000) {
    requestCyclic(0x40, STATUS_REQ_INTERVAL_MS);
    requestCyclic(0x30, RPM_REQ_INTERVAL_MS);
    tLastReissue = millis();
  }

  // buzzer off timer
  if (tBuzzerOff && millis() >= tBuzzerOff) {
    if (BUZZER_PIN >= 0)
      digitalWrite(BUZZER_PIN, LOW);
    tBuzzerOff = 0;
  }

  // manage RTD entry
  if (!readyToDrive) {
    tryEnterRTD();
  }
}
Motor::Motor(bool *brake_active) {
  this->brake_active = brake_active;
  if (DEBUG_MODE)
    Serial.println("Motor Controller init");

  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);
  if (BUZZER_PIN >= 0) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // CAN
  Can1.begin();
  Can1.setBaudRate(BAUDRATE);
  delay(50);
  Serial.println("CAN bus initialized");

  clearErrors();
  setCanTimeout(CAN_TIMEOUT);
  delay(50);
  Serial.println("Cleared errors and set CAN timeout");

  // set cyclic requests
  requestCyclic(ADDRS::STATUS, STATUS_REQ_INTERVAL_MS); // status
  requestCyclic(ADDRS::RPM, RPM_REQ_INTERVAL_MS);       // rpm
  // one shot dc bus
  // request_once(ADDRS::READ_DC);
  send3(ADDRS::READ, ADDRS::READ_DC, 0);

  tLastReissue = millis();
}

Motor::MotorResponse Motor::setTorque(double desired) {

  if (desired < double{0}) {
    return MotorResponse::NEGATIVE_TORQUE;
  }
  if (desired > this->MAX_TORQUE) {
    return MotorResponse::OVERTORQUE;
  }
  if (!readyToDrive) {
    return MotorResponse::DISABLED;
  }
  if (faultActive) {
    return MotorResponse::CAN_ERROR;
  }
  if (desired != lastTorque) {
    setTorqueRaw(desired);
    lastTorque = desired;
  }
}

void Motor::set(int function, int val) {
  send3(function, val & 0xFF, (val >> 8) & 0xFF);
}

// I don't know how to handle this right now, hence the comment
/*uint16_t Motor::request_once(int field) {
  send3(ADDRS::READ, field, 0x00);
  CAN_message_t msg;

  while (Can1.read(msg)) {
    if (msg.id != BAMOCAR_TX_ID || msg.len < 3)
      continue;
    const uint8_t reg = msg.buf[0];
    if (reg == field)
  }
}
*/

bool Motor::getWarning(uint8_t field) {
  return getFromBitfield(ADDRS::WARNING_ERROR, field).value_or(false);
}

void Motor::clearErrors() {
  // TODO: This needs testing. I don't know the behvior of the errors
  set(ADDRS::CANCEL_ERRORS, 1);
}

void Motor::lockDrive() {
  setTorque(double{0});
  // Set the lock drive bit in the control register
  set(ADDRS::LOCK_DRIVE, 1 << ADDRS::LOCK_DRIVE_BIT);
}
