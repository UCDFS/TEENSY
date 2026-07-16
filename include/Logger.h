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
  static const char* _driverName;

  // Persistent boot counter (session.cnt on the SD card) - no RTC on this
  // board, so session numbers are the only way to tell one power-on's files
  // apart from the next. Reads the stored value (1 if the file doesn't exist
  // yet), immediately writes back n+1, and returns n - so even if this
  // session never shuts down cleanly, the number is already consumed and the
  // next boot won't collide with it.
  static uint32_t nextSessionNumber();

public:
  static CircularBuffer<LogEntry, MAX_BUF> _logBuffer;
  static bool begin();
  static void sync();
  static void setDriver(const char* name);
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