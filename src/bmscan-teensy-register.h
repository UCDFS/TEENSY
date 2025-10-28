#ifndef BMSCAN_TEENSY_REGISTER_H
#define BMSCAN_TEENSY_REGISTER_H

// ----------------------------------------------
// Orion BMS 2 - CAN Communication Register Map
// ----------------------------------------------

// --- CAN Settings ---
#define BMS_RX_ID           0x7E3   // ID the BMS listens on (example)
#define BMS_TX_ID           0x7E4   // ID the BMS transmits from (example)
#define BMS_BAUD_RATE       500000  // Standard Formula Student CAN bus rate

// --- Common CAN IDs for Orion BMS 2 ---
#define BMS_CAN_ID_PACK_VOLTAGE      0x400  // Total pack voltage
#define BMS_CAN_ID_PACK_CURRENT      0x401  // Pack current (positive = discharge)
#define BMS_CAN_ID_PACK_SOC          0x402  // State of Charge (%)
#define BMS_CAN_ID_TEMPERATURES      0x403  // Min/Max/Average cell temps
#define BMS_CAN_ID_CELL_VOLTAGES     0x404  // Min/Max/Average cell voltages
#define BMS_CAN_ID_STATUS_FLAGS      0x405  // Status / fault flags
#define BMS_CAN_ID_BALANCING_STATUS  0x406  // Cell balancing states
#define BMS_CAN_ID_VOLT_LIMITS       0x407  // Charge/discharge voltage limits
#define BMS_CAN_ID_CURRENT_LIMITS    0x408  // Charge/discharge current limits

// --- Request intervals (same logic as Bamocar) ---
#define INTVL_IMMEDIATE   0x00
#define INTVL_SUSPEND     0xFF
#define INTVL_100MS       0x64
#define INTVL_200MS       0xC8
#define INTVL_500MS       0xFA

#endif
