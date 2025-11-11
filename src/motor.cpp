#include "motor.h"
#include "units.h"

// Static member definitions
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Motor::Can1;
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

void Motor::handleStatusWord(uint16_t word) {
  Motor::statusWord = word;
  Motor::faultActive = (word & 0x0040) != 0;
  const bool enableBit = (word & 0x0001) != 0;
  const bool readyBit = (word & 0x0004) != 0;

  if (Motor::faultActive) {
    if (Motor::readyToDrive && DEBUG_MODE)
      Serial.println("Inverter fault. RTD cleared.");
    Motor::readyToDrive = false;
    if (Motor::rtdRequestPending && DEBUG_MODE)
      Serial.println("Pending RTD cancelled due to fault.");
    Motor::rtdRequestPending = false;
    Motor::tPendingReady = 0;
    if (Motor::tBuzzerOff) {
      if (BUZZER_PIN >= 0)
        digitalWrite(BUZZER_PIN, LOW);
      Motor::tBuzzerOff = 0;
    }
    return;
  }

  if (enableBit && readyBit) {
    if (!Motor::readyToDrive) {
      Motor::readyToDrive = true;
      Motor::rtdRequestPending = false;
      Motor::tPendingReady = 0;
      if (BUZZER_PIN >= 0) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, HIGH);
        Motor::tBuzzerOff = millis() + 3000;
      }
      if (DEBUG_MODE)
        Serial.println("Inverter confirmed RTD.");
    }
  } else if (Motor::readyToDrive) {
    Motor::readyToDrive = false;
    Motor::tPendingReady = 0;
    if (Motor::tBuzzerOff) {
      if (BUZZER_PIN >= 0)
        digitalWrite(BUZZER_PIN, LOW);
      Motor::tBuzzerOff = 0;
    }
    if (DEBUG_MODE)
      Serial.println("Inverter exited RTD state.");
  }
}

void Motor::readCAN() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Logging::logCANFrame(msg, "RX");
    if (msg.id != BAMOCAR_TX_ID || msg.len < 3)
      continue;
    const uint8_t reg = msg.buf[0];

    if (reg == 0x40) { // status word
      const uint16_t word = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      this->handleStatusWord(word);
    } else if (reg == 0x30) { // rpm
      rpmFeedback = int16_t(uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8));
    } else if (reg == 0xEB) { // dc bus voltage 0.1 V/LSB
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

  if (!brakeActive() || !rtdButtonPressed()) {
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
namespace MotorController {

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

void setTorque(int16_t torqueCounts) {
  if (!readyToDrive || faultActive)
    return;
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

Motor::Motor() {
  if (DEBUG_MODE)
    Serial.println("Motor Controller init");

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
  requestCyclic(ADDRS::STATUS, STATUS_REQ_INTERVAL_MS); // status
  requestCyclic(ADDRS::RPM, RPM_REQ_INTERVAL_MS);       // rpm
  // one shot dc bus
  request_once(ADDRS::READ_DC);

  tLastReissue = millis();
}

Motor::MotorResponse Motor::set_torque(units::torque::newton_meter_t desired) {
  if (desired < units::torque::newton_meter_t{0}) {
    return MotorResponse::NEGATIVE_TORQUE;
  }
  if (desired > this->MAX_TORQUE) {
    return MotorResponse::OVERTORQUE;
  }
  this->sendTorqueCommand(desired.value());
}

void Motor::set(int function, int val) {
  send3(function, val & 0xFF, (val >> 8) & 0xFF);
}
void Motor::request_once(int field) { send3(ADDRS::READ, field, 0x00); }
