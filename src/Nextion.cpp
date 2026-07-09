#include "Nextion.h"

void Nextion::sendCmd(const char *cmd) {
  while (NEXTION_SERIAL.available()) NEXTION_SERIAL.read();
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
  // isbr must be on for t_systems' multi-row status text to honour the \r
  // line breaks in sendText() - forced here so it doesn't depend on the
  // HMI project having saved that attribute correctly.
  sendCmd(NX_BOOT_SYSTEMS ".isbr=1");
  sendCmd("page 0");
}

void Nextion::page(uint8_t pageNumber) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "page %d", pageNumber);
  sendCmd(cmd);
}

void Nextion::sendText(const char *component, const char *text) {
  char cmd[160];  // t_systems' multi-row status text can run past the old 80
  snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component, text);
  sendCmd(cmd);
}

void Nextion::sendNumber(const char *component, int16_t value) {
  char cmd[32];
  char buf[8];
  itoa(value, buf, 10);
  snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component, buf);
  sendCmd(cmd);
}

void Nextion::sendNumberValue(const char *component, int16_t value) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "%s.val=%d", component, value);
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
  // t_drive (NX_DRIVE_STATE) is NOT sent here - main.cpp's state machine
  // already writes richer text there directly (hold progress bar, CLEARED,
  // HOLD BRAKE, etc). Sending dashStatus.driveOn here too would race it.
  sendNumber(NX_DRIVE_MOTOR_TEMP, dashStatus.motorTemp);
  sendNumber(NX_DRIVE_INVERTER_TEMP, dashStatus.inverterTemp);
  sendNumberValue(NX_DRIVE_SPEED_BAR, dashStatus.torque);
}