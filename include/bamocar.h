#pragma once
#include "logger.h"
#include "config.h"

#define REG_TRANSMIT_REQUEST 0x3D
#define REG_STATUS           0x40
#define REG_SPEED_ACTUAL     0x30
#define REG_CURRENT_ACTUAL   0x20
#define REG_TEMP_MOTOR       0x49
#define REG_TEMP_INVERTER    0x4A
#define REG_DC_BUS_VOLTAGE   0xEB
#define REG_ERROR_WORD       0x8F
#define REG_CLEAR_ERRORS     0x8E
#define REG_CAN_TIMEOUT      0xD0
#define REG_DRIVE_COMMAND    0x51
#define REG_TORQUE_COMMAND   0x90

#define BAMOCAR_RX_ID 0x201  // Teensy → Bamocar
#define BAMOCAR_TX_ID 0x181  // Bamocar → Teensy

namespace CAN 
{
void requestStatusCyclic(uint8_t interval_ms);
void requestErrorsCyclic(uint8_t interval_ms);
void requestStatusOnce();
void requestSpeedCyclic(uint8_t interval_ms);
void requestCurrentCyclic(uint8_t interval_ms);
void requestTempsCyclic(uint8_t interval_ms);
void requestDCBusOnce();
void clearErrors();
void enableDrive();
void disableDrive();
void sendTorqueCommand(int16_t torqueValue);
void configureCanTimeout(uint16_t ms);
void sendCAN(const CAN_message_t &msg);
void readCanMessages();
void bamocarErrorDescription(uint32_t errorWord, char *buf, size_t len);
}