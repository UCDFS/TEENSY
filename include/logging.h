/**
 * @file logging.h
 * @brief SD-card logging interface for CAN traffic capture.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>
#include <SD.h>
#include <FlexCAN_T4.h>

// Logs CAN frames, generates filenames, and streams to Serial2 (ESP)
namespace Logging {
  void init();                              // Mount SD + open new log file
  void logCANFrame(const CAN_message_t &msg, const char *dir);
  void flush();
  void close();
}
