#include "Nextion.h"

void Nextion::sendCmd(const char *cmd) {
  NEXTION_SERIAL.print(cmd);
  NEXTION_SERIAL.write(0xFF); // Nextion command terminator
  NEXTION_SERIAL.write(0xFF);
  NEXTION_SERIAL.write(0xFF);
}

void Nextion::begin() {
  NEXTION_SERIAL.begin(NEXTION_BAUD);
  delay(100); 
  sendCmd(""); // clear RX buffer
  sendCmd("bkcmd=1"); // enable Nextion command feedback
  sendCmd("page 0");
}

void Nextion::page(uint8_t pageNumber) {
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "page %d", pageNumber);
  sendCmd(cmd);
}

void Nextion::sendText(const char *component, const char *text) {
  char cmd[80];
  snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component, text);
  sendCmd(cmd);
}

void Nextion::sendNumber(const char *component, uint16_t value) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "%s.val=%u", component, value);
  sendCmd(cmd);
}

void Nextion::bootStatus(const char *phase, const char *detail) {
  sendText(NX_BOOT_STATUS, phase);
  if (detail[0] != '\0') sendText(NX_BOOT_DETAIL, detail);
}

void Nextion::updateDash(DashStatus dashStatus) {
  sendNumber(NX_DRIVE_SPEED, dashStatus.speed);
  sendNumber(NX_DRIVE_RPM, dashStatus.rpm);
  sendNumber(NX_DRIVE_TORQUE, dashStatus.torque);
  sendNumber(NX_DRIVE_DCBUS, dashStatus.dcBusV);
  sendText(NX_DRIVE_FAULT, dashStatus.fault ? "FAULT" : "OK");
  sendText(NX_DRIVE_STATE, dashStatus.driveOn ? "DRIVE: ON" : "DRIVE: OFF");
  sendNumber(NX_DRIVE_MTEMP, dashStatus.mTemp);
  sendNumber(NX_DRIVE_ITEMP, dashStatus.iTemp);
}