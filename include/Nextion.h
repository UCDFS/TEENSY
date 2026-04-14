#pragma once
#include "config.h"

// Nextion connected to Serial7 (TX7/RX7 on Teensy 4.1)
#define NEXTION_SERIAL Serial7
#define NEXTION_BAUD   115200

// ---- Page IDs ----
#define NX_PAGE_BOOT   0
#define NX_PAGE_DRIVE  1

// ---- Boot page component names (from Nextion Editor) ----
#define NX_BOOT_STATUS "t_status"   // text: current phase
#define NX_BOOT_DETAIL "t_detail"   // text: secondary detail

// ---- Drive page component names ----
#define NX_DRIVE_SPEED  "n_speed"   // number: vehicle speed, integer km/h
#define NX_DRIVE_RPM    "n_rpm"     // number: motor RPM
#define NX_DRIVE_TORQUE "n_torque"  // number: torque command, 0-100%
#define NX_DRIVE_DCBUS  "n_dcbus"   // number: DC bus voltage, whole volts
#define NX_DRIVE_FAULT  "t_fault"   // text: "OK" or "FAULT"
#define NX_DRIVE_STATE  "t_drive"   // text: "DRIVE: ON" or "DRIVE: OFF"
#define NX_DRIVE_MOTOR_TEMP  "n_mtemp"   // number: motor temperature, °C
#define NX_DRIVE_INVERTER_TEMP  "n_itemp"   // number: inverter temperature, °C

struct DashStatus {
  int16_t speed; // km/h
  int16_t rpm;
  int16_t torque; 
  int16_t dcBusV; 
  bool fault;     
  bool driveOn;   
  int16_t motorTemp;  
  int16_t inverterTemp;
};

class Nextion {
private:
  static void sendCmd(const char *cmd);
public:
  static void begin();
  static void page(uint8_t pageNumber);
  static void sendText(const char *component, const char *text);
  static void sendNumber(const char *component, int16_t value);
  static void bootStatus(const char *phase, const char *detail);
  static void updateDash(DashStatus dashStatus);
};
