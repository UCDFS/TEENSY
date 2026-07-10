#pragma once
#include "config.h"
#include "CircularBuffer.h"

enum class LogLevel { NONE, ERROR, WARNING, INFO, DEBUG };


struct LogEntry {
    char data[MAX_LOG_LEN];
};

class Logger {
private:
  static SdFs sd;
  static FsFile logFile;
  static FsFile telemetryFile;

public:
  static CircularBuffer<LogEntry, MAX_BUF> _logBuffer;
  static bool begin();
  static void sync();
  static void log(LogLevel level, const char* module, const char* msg);
  static void logCANFrame(const CAN_message_t &msg, const char *dir);
  static void process();
  static const char* levelToStr(LogLevel level);

  // Fixed-rate structured channel log (telemetry.csv) - separate from the
  // free-text/CAN-frame buffer above so a burst of CAN traffic can't crowd
  // out or get crowded out by this stream. Caller (emitTelemetry) is already
  // rate-limited to 20Hz, so this writes directly with no intermediate queue.
  static void writeTelemetryHeader(const char *header);
  static void writeTelemetryRow(const char *csvLine);
};