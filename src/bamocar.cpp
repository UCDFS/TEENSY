#include "bamocar.h"

namespace CAN 
{

void sendCAN(const CAN_message_t &msg) {
  Can1.write(msg);
  Logger::logCANFrame(msg, "TX");
}

void requestStatusCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_STATUS;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void requestErrorsCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_ERROR_WORD;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void requestStatusOnce() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_STATUS;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void requestSpeedCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_SPEED_ACTUAL;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void requestCurrentCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_CURRENT_ACTUAL;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void requestTempsCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_TEMP_MOTOR;  // motor temp
  msg.buf[2] = interval_ms;
  sendCAN(msg);
  msg.buf[1] = REG_TEMP_INVERTER;  // inverter (IGBT) temp
  sendCAN(msg);
}

void requestDCBusOnce() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TRANSMIT_REQUEST;
  msg.buf[1] = REG_DC_BUS_VOLTAGE;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void clearErrors() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_CLEAR_ERRORS;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void configureCanTimeout(uint16_t ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_CAN_TIMEOUT;
  msg.buf[1] = ms & 0xFF;
  msg.buf[2] = (ms >> 8) & 0xFF;
  sendCAN(msg);
}

void enableDrive() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_DRIVE_COMMAND;
  msg.buf[1] = 0x04;
  msg.buf[2] = 0x00;
  sendCAN(msg);
  delay(100);
  msg.buf[1] = 0x00;
  sendCAN(msg);
}

void disableDrive() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_DRIVE_COMMAND;
  msg.buf[1] = 0x04;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void sendTorqueCommand(int16_t torqueValue) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = REG_TORQUE_COMMAND;
  msg.buf[1] = torqueValue & 0xFF;
  msg.buf[2] = (torqueValue >> 8) & 0xFF;
  sendCAN(msg);
}

// ---------- Error word lookup (RegID 0x8F, BAMOCAR-PG-D3 Manual) ----------
static const char *const ERROR_NAMES[16] = {
  "BADPARAS",     // bit 0  - Parameter damaged
  "POWERFAULT",   // bit 1  - Hardware error
  "RFE",          // bit 2  - Safety circuit faulty
  "BUS TIMEOUT",  // bit 3  - CAN timeout exceeded
  "FEEDBACK",     // bit 4  - Bad/wrong encoder signal
  "POWERVOLTAGE", // bit 5  - Power voltage missing
  "MOTORTEMP",    // bit 6  - Engine temperature too high
  "DEVICETEMP",   // bit 7  - Unit temperature too high
  "OVERVOLTAGE",  // bit 8  - Overvoltage
  "I_PEAK",       // bit 9  - Overcurrent
  "RACEAWAY",     // bit 10 - Spinning
  "USER",         // bit 11 - User error selection
  "",             // bit 12 - (unmapped)
  "",             // bit 13 - (unmapped)
  "HW_ERR",       // bit 14 - Current measurement error
  "BALLAST"       // bit 15 - Ballast circuit overloaded
};

void bamocarErrorDescription(uint32_t errorWord, char *buf, size_t len) {
  for (int bit = 0; bit < 16; bit++) {
    if (errorWord & (1u << bit)) {
      const char *name = (ERROR_NAMES[bit][0]) ? ERROR_NAMES[bit] : "UNKNOWN";
      strncpy(buf, name, len - 1);
      buf[len - 1] = '\0';
      return;
    }
  }
  strncpy(buf, "UNKNOWN", len - 1);
  buf[len - 1] = '\0';
}

// ---------- CAN RX ----------
void readCanMessages() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    Logger::logCANFrame(msg, "RX");

    if (msg.id == BAMOCAR_TX_ID && msg.len >= 3) {
      lastBAMOCARRx = millis();
      uint8_t reg = msg.buf[0];

      if (reg == REG_STATUS) { // STATUS register
        bamocarOnline = true;
        statusWord = (int16_t)(msg.buf[1] | (msg.buf[2] << 8));
      }

      else if (reg == REG_SPEED_ACTUAL) { // RPM feedback (signed, normalised to NMAX)
        rpmFeedback = (int16_t)(msg.buf[1] | (msg.buf[2] << 8));
      }

      else if (reg == REG_CURRENT_ACTUAL) { // I_ACT actual current (signed, normalised to IMAX)
        actualCurrent = (int16_t)(msg.buf[1] | (msg.buf[2] << 8));
      }

      else if (reg == 0x49) { // motor temperature (°C)
        uint16_t raw = msg.buf[1] | (msg.buf[2] << 8);
        motorTemp = motorADCToTemp(raw);
      }

      else if (reg == 0x4A) { // inverter (IGBT) temperature (°C)
        uint16_t raw = msg.buf[1] | (msg.buf[2] << 8);
        inverterTemp = igbtADCToTemp(raw);
      }

      else if (reg == REG_DC_BUS_VOLTAGE) { // DC bus voltage (UDC = raw / 31.5848, per BAMOCAR FAQ)
        dcBusVoltage = (msg.buf[1] | (msg.buf[2] << 8)) / 31.5848f;
      }

      else if (reg == REG_ERROR_WORD) { // Error register
        bamocarErrorWord = msg.buf[1] | (msg.buf[2] << 8);
      }
    }
  }
}

}