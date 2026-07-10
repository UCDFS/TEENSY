#include "bamocar.h"
#include "Bms.h"

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

// BAMOCAR FAQ Q10 ("How to configure CAN-Time-Out..."): a CAN-timeout
// (BUS TIMEOUT) fault can only be erased while the drive is locked - the
// plain 0x8E clear-errors message alone does nothing for it. Unlocking
// (the Lock->Enable sequence from Q9) is what actually erases it, so
// clear -> lock -> enable -> immediately re-lock, leaving the drive
// exactly as off as it was before a bare RTD press called this.
void clearErrorsSequence() {
  clearErrors();
  enableDrive();
  delay(100);
  disableDrive();
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

// ---------- Temperature conversion (IGBT + motor) ----------
// Shared linear-interpolation lookup, clamped at the table ends. Both curves
// below use it - factored out now that there are two nearly-identical tables
// instead of duplicating the same scan/interpolate logic twice.
struct TempPoint { float x; float degC; };

static float interpTempTable(const TempPoint *table, size_t len, float x) {
  if (x <= table[0].x) return table[0].degC;
  if (x >= table[len - 1].x) return table[len - 1].degC;

  for (size_t i = 1; i < len; i++) {
    if (x <= table[i].x) {
      const TempPoint &lo = table[i - 1];
      const TempPoint &hi = table[i];
      float frac = (x - lo.x) / (hi.x - lo.x);
      return lo.degC + frac * (hi.degC - lo.degC);
    }
  }
  return table[len - 1].degC;  // unreachable
}

// BAMOCAR-PG-D3-700/400 manual p.49, "6.2 Power stages - temperature":
// 3x NTC in IGBT, register 0x4A. Raw ADC count vs degC (table spans -30 to 125 C).
static const TempPoint IGBT_TEMP_TABLE[] = {
  {16308, -30}, {16387, -25}, {16487, -20}, {16609, -15}, {16757, -10},
  {16938,  -5}, {17151,   0}, {17400,   5}, {17688,  10}, {18017,  15},
  {18387,  20}, {18797,  25}, {19247,  30}, {19733,  35}, {20250,  40},
  {20793,  45}, {21357,  50}, {21933,  55}, {22515,  60}, {23097,  65},
  {23671,  70}, {24232,  75}, {24775,  80}, {25296,  85}, {25792,  90},
  {26261,  95}, {26702, 100}, {27114, 105}, {27497, 110}, {27851, 115},
  {28179, 120}, {28480, 125},
};
#define IGBT_TEMP_TABLE_LEN (sizeof(IGBT_TEMP_TABLE) / sizeof(IGBT_TEMP_TABLE[0]))

float igbtADCToTemp(uint16_t raw) {
  return interpTempTable(IGBT_TEMP_TABLE, IGBT_TEMP_TABLE_LEN, (float)raw);
}

// KTY81-210 resistance -> temperature (EMRAX 208's motor temp sensor, per
// EMRAX datasheet). R25=2000 ohm matches the sensor's datasheet-defining
// spec exactly, corroborating this curve.
static const TempPoint KTY_TEMP_TABLE[] = {
  { 980, -55}, {1030, -50}, {1135, -40}, {1247, -30}, {1367, -20},
  {1495, -10}, {1630,   0}, {1772,  10}, {1922,  20}, {2000,  25},
  {2080,  30}, {2245,  40}, {2417,  50}, {2597,  60}, {2785,  70},
  {2980,  80}, {3182,  90}, {3392, 100}, {3607, 110}, {3817, 120},
  {3915, 125}, {4008, 130}, {4166, 140}, {4280, 150},
};
#define KTY_TEMP_TABLE_LEN (sizeof(KTY_TEMP_TABLE) / sizeof(KTY_TEMP_TABLE[0]))

// BAMOCAR-PG-D3-700/400 manual, X2 connector schematic (resolver + motor
// temp): VCC+5V -> 4k7 pull-up -> TEMP node (100nF filter) -> pin H -> KTY
// sensor -> pin L -> GND. Series-R (4700 ohm) is confirmed straight off that
// schematic. ADC full-scale (32768) is NOT confirmed anywhere in the docs -
// assumed consistent with BAMOCAR's other signed 16-bit normalised registers
// (speed/current both use +-32767 full scale). If motor temp reads
// implausibly, this full-scale assumption is the first thing to re-check.
float motorADCToTemp(uint16_t raw) {
  const float kSeriesOhm = 4700.0f;
  const float kADCMax = 32768.0f;
  float resistance = kSeriesOhm * (float)raw / (kADCMax - (float)raw);
  return interpTempTable(KTY_TEMP_TABLE, KTY_TEMP_TABLE_LEN, resistance);
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
      lastBamocarRx = millis();
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

      else if (reg == 0x49) { // motor temperature (KTY81-210, see motorADCToTemp above)
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

    else if (Bms::isBmsFrame(msg.id)) {
      Bms::handleFrame(msg);
    }
  }
}

}