/**
 * @file logging.cpp
 * @brief Provides SD-card based logging utilities for CAN traffic.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#include "logging.h"

static File logFile;
static bool sdAvailable = false;
static const int chipSelect = BUILTIN_SDCARD;

static String generateFilename() {
  int index = 1;
  char filename[32];
  do {
    sprintf(filename, "CAN_traffic_logs_%04d.csv", index);
    index++;
  } while (SD.exists(filename));
  return String(filename);
}

void Logging::init() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card init failed!");
    sdAvailable = false;
    return;
  }

  String filename = generateFilename();
  logFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open log file!");
    sdAvailable = false;
    return;
  }

  logFile.println("Time(ms),Dir,ID,Len,B0,B1,B2,B3,B4,B5,B6,B7,Decoded");
  logFile.flush();
  sdAvailable = true;

  Serial.print("Logging to: ");
  Serial.println(filename);
}

void Logging::logCANFrame(const CAN_message_t &msg, const char *dir) {
  if (!sdAvailable) return;
  noInterrupts();
  logFile.printf("%lu,%s,0x%03X,%d", millis(), dir, msg.id, msg.len);
  if (msg.len > 0) logFile.printf(",0x%02X", msg.buf[0]);
  else logFile.print(",");
  for (int i = 0; i < 8; i++) {
    if (i < msg.len) logFile.printf(",0x%02X", msg.buf[i]);
    else logFile.print(",");
  }

  // Optional: interpret known Bamocar regs
  uint8_t reg = msg.buf[0];
  char decoded[64];
  if (msg.id == 0x201) {
    sprintf(decoded, "TX reg 0x%02X", reg);
  } else if (msg.id == 0x181) {
    sprintf(decoded, "RX reg 0x%02X", reg);
  } else {
    sprintf(decoded, "ID 0x%03X", msg.id);
  }

  logFile.printf(",\"%s\"\r\n", decoded);
  logFile.flush();

  // Stream to Serial2 for ESP telemetry
  Serial2.printf("CAN:%s,0x%03X,%d", dir, msg.id, msg.len);
  for (int i = 0; i < msg.len; i++) Serial2.printf(",0x%02X", msg.buf[i]);
  Serial2.println();
}

void Logging::flush() {
  if (sdAvailable) logFile.flush();
}

void Logging::close() {
  if (sdAvailable) logFile.close();
}
