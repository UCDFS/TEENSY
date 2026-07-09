#pragma once
#include "config.h"

// BMS_Firmware_KR drive-CAN broadcast (see bms_can.h/.c in that repo):
// 500 kbit/s, standard 11-bit IDs, BMS is TX-only on this bus - Teensy only
// ever receives. Frame map, base = BMS_CAN_BASE_ID (must match the BMS's
// configured can_base_id, default 0x500):
//   base+0x000  Pack status   100ms  vbat_mv,vpack_mv,i_batt_ma,state,outputs/flags
//   base+0x001  Fault status  100ms  active_faults[31:0], latched_faults[31:0]
//   base+0x010..0x022 (19)    100ms  cell voltages, 4/frame, uint16 mV, 0xFFFF=invalid
//   base+0x030..0x042 (19)    500ms  temps, 4/frame, int16 degC x10, 0x8000=invalid
// Only pack status + faults are parsed structurally; cell/temp bursts are
// scanned for running min/max rather than stored per-cell (75+75 values
// aren't needed on the VCU side, just pack health margins).

#define BMS_OFF_PACK      0x000
#define BMS_OFF_FAULTS    0x001
#define BMS_OFF_CELLS_LO  0x010
#define BMS_OFF_CELLS_HI  0x022
#define BMS_OFF_TEMPS_LO  0x030
#define BMS_OFF_TEMPS_HI  0x042

#define BMS_TEMP_INVALID_CX10  ((int16_t)0x8000)  // matches bms_types.h TEMP_INVALID_CX10

// BmsState (bms_state.h) - mirrored here, this firmware doesn't own it.
enum class BmsState : uint8_t {
  INIT = 0, STANDBY = 1, DISCHARGE = 2, CHARGE = 3, FAULT = 4, SHUTDOWN = 5
};

extern bool bmsOnline;
extern uint32_t lastBmsRx;

extern int32_t bmsVbatMv;
extern int32_t bmsVpackMv;
extern int32_t bmsIBattMa;      // positive = discharge
extern bool bmsVbatValid;
extern bool bmsVpackValid;
extern bool bmsIBattValid;

extern uint8_t bmsStateRaw;     // raw BmsState byte
extern bool bmsMasterOk;
extern bool bmsDischargeOk;
extern bool bmsChargeOk;
extern bool bmsChargerSafetyOk;

extern uint32_t bmsActiveFaults;   // bit layout: protocol/fault_bits.yaml
extern uint32_t bmsLatchedFaults;

extern uint16_t bmsCellMinMv;      // 0xFFFF = no valid cell frame seen yet
extern uint16_t bmsCellMaxMv;
extern int16_t bmsTempMinCx10;     // BMS_TEMP_INVALID_CX10 = no valid temp frame seen yet
extern int16_t bmsTempMaxCx10;

namespace Bms {
bool isBmsFrame(uint32_t id);
void handleFrame(const CAN_message_t &msg);
const char *stateName(uint8_t state);
void faultDescription(uint32_t faultBits, char *buf, size_t len);  // first active bit's name
}
