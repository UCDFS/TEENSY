#pragma once
#include "config.h"
#include "CircularBuffer.h"

enum LogLevel { DEBUG, ERROR, INFO, WARNING };

// In C++, this is the cleaner way to define a fixed-size char array type
struct LogEntry {
    char data[MAX_LOG_LEN];
};

class Logger {
private:
  static SdFs sd;         // The SD filesystem object
  static FsFile logFile;  // The log file object
    
public:
  static CircularBuffer<LogEntry, MAX_BUF> _logBuffer;
  static bool begin();
  static void sync();
  static void log(LogLevel level, const char* module, const char* msg);
  static void process();
  static const char* levelToStr(LogLevel level);
};