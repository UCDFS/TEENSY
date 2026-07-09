#include "Bms.h"

bool bmsOnline = false;
uint32_t lastBmsRx = 0;

int32_t bmsVbatMv = 0;
int32_t bmsVpackMv = 0;
int32_t bmsIBattMa = 0;
bool bmsVbatValid = false;
bool bmsVpackValid = false;
bool bmsIBattValid = false;

uint8_t bmsStateRaw = 0;
bool bmsMasterOk = false;
bool bmsDischargeOk = false;
bool bmsChargeOk = false;
bool bmsChargerSafetyOk = false;

uint32_t bmsActiveFaults = 0;
uint32_t bmsLatchedFaults = 0;

uint16_t bmsCellMinMv = 0xFFFF;   // sentinel doubles as the wire's own "invalid" marker
uint16_t bmsCellMaxMv = 0;
int16_t bmsTempMinCx10 = BMS_TEMP_INVALID_CX10;
int16_t bmsTempMaxCx10 = BMS_TEMP_INVALID_CX10;

namespace Bms {

bool isBmsFrame(uint32_t id) {
  return id >= BMS_CAN_BASE_ID && id <= (BMS_CAN_BASE_ID + BMS_OFF_TEMPS_HI);
}

// bms_can.c _send_pack(): buf[0-1]=vbat_mv, [2-3]=vpack_mv, [4-5]=i_batt (100mA/LSB,
// signed, positive=discharge), [6]=state, [7]=flags (bits[3:0]=outputs, bits[6:4]=validity)
static void handlePack(const CAN_message_t &msg) {
  if (msg.len < 8) return;
  uint16_t vbatRaw  = msg.buf[0] | (msg.buf[1] << 8);
  uint16_t vpackRaw = msg.buf[2] | (msg.buf[3] << 8);
  int16_t  iRaw     = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
  uint8_t  flags    = msg.buf[7];

  bmsVbatValid  = (flags & 0x10) != 0;
  bmsVpackValid = (flags & 0x20) != 0;
  bmsIBattValid = (flags & 0x40) != 0;

  bmsVbatMv  = bmsVbatValid  ? (int32_t)vbatRaw  : 0;
  bmsVpackMv = bmsVpackValid ? (int32_t)vpackRaw : 0;
  bmsIBattMa = bmsIBattValid ? (int32_t)iRaw * 100 : 0;

  bmsStateRaw        = msg.buf[6];
  bmsMasterOk        = (flags & 0x01) != 0;
  bmsDischargeOk     = (flags & 0x02) != 0;
  bmsChargeOk        = (flags & 0x04) != 0;
  bmsChargerSafetyOk = (flags & 0x08) != 0;
}

// bms_can.c _send_faults(): buf[0-3]=active_faults, buf[4-7]=latched_faults, both LE u32
static void handleFaults(const CAN_message_t &msg) {
  if (msg.len < 8) return;
  bmsActiveFaults  = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8) |
                      ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);
  bmsLatchedFaults = (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) |
                      ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);
}

// 4 cells/frame, uint16 mV, 0xFFFF=invalid - tracked as running min/max only,
// not stored per-cell (VCU needs pack margins, not per-cell diagnostics).
static void handleCellFrame(const CAN_message_t &msg) {
  for (int i = 0; i < 4 && (size_t)(i * 2 + 1) < msg.len; i++) {
    uint16_t mv = msg.buf[i * 2] | (msg.buf[i * 2 + 1] << 8);
    if (mv == 0xFFFF) continue;
    if (mv < bmsCellMinMv) bmsCellMinMv = mv;
    if (mv > bmsCellMaxMv) bmsCellMaxMv = mv;
  }
}

// 4 temps/frame, int16 degC x10, 0x8000=invalid - same running min/max approach.
static void handleTempFrame(const CAN_message_t &msg) {
  for (int i = 0; i < 4 && (size_t)(i * 2 + 1) < msg.len; i++) {
    int16_t cx10 = (int16_t)(msg.buf[i * 2] | (msg.buf[i * 2 + 1] << 8));
    if (cx10 == BMS_TEMP_INVALID_CX10) continue;
    if (bmsTempMinCx10 == BMS_TEMP_INVALID_CX10 || cx10 < bmsTempMinCx10) bmsTempMinCx10 = cx10;
    if (bmsTempMaxCx10 == BMS_TEMP_INVALID_CX10 || cx10 > bmsTempMaxCx10) bmsTempMaxCx10 = cx10;
  }
}

void handleFrame(const CAN_message_t &msg) {
  bmsOnline = true;
  lastBmsRx = millis();

  uint32_t off = msg.id - BMS_CAN_BASE_ID;
  if (off == BMS_OFF_PACK) handlePack(msg);
  else if (off == BMS_OFF_FAULTS) handleFaults(msg);
  else if (off >= BMS_OFF_CELLS_LO && off <= BMS_OFF_CELLS_HI) handleCellFrame(msg);
  else if (off >= BMS_OFF_TEMPS_LO && off <= BMS_OFF_TEMPS_HI) handleTempFrame(msg);
}

const char *stateName(uint8_t state) {
  switch ((BmsState)state) {
    case BmsState::INIT:      return "INIT";
    case BmsState::STANDBY:   return "STANDBY";
    case BmsState::DISCHARGE: return "DISCHARGE";
    case BmsState::CHARGE:    return "CHARGE";
    case BmsState::FAULT:     return "FAULT";
    case BmsState::SHUTDOWN:  return "SHUTDOWN";
  }
  return "UNKNOWN";
}

// protocol/fault_bits.yaml bit order (bms_types.h FaultBit enum), bits 0-22.
// Short mnemonics, not the full ui_text sentences - dash pill space is tight.
static const char *const FAULT_NAMES[23] = {
  "CELL_OV", "CELL_UV", "CELL_OV_SOFT", "CELL_UV_SOFT", "CELL_INVALID",
  "CELL_OPENWIRE", "TEMP_OVER_CHG", "TEMP_OVER_DISCHG", "TEMP_OVER_ABS",
  "TEMP_INVALID", "TEMP_COVERAGE", "VBAT_INVALID", "VPACK_INVALID",
  "ISOSPI_CELL", "ISOSPI_TEMP", "I2C_ISL28022", "WATCHDOG", "CONFIG_INVALID",
  "OVERCURRENT", "BAL_TEMP_VIOL", "TEMP_CHAIN_BAL_ATTEMPT", "TEMP_COLD_CHG",
  "TEMP_COLD_DISCHG",
};

void faultDescription(uint32_t faultBits, char *buf, size_t len) {
  for (int bit = 0; bit < 23; bit++) {
    if (faultBits & (1u << bit)) {
      strncpy(buf, FAULT_NAMES[bit], len - 1);
      buf[len - 1] = '\0';
      return;
    }
  }
  strncpy(buf, "OK", len - 1);
  buf[len - 1] = '\0';
}

}
