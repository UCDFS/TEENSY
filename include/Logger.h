#pragma once
#include "config.h"
#include "CircularBuffer.h"

enum LogLevel { NONE, ERROR, WARNING, INFO, DEBUG };


struct LogEntry {
    char data[MAX_LOG_LEN];
};

class Logger {
private:
  static SdFs sd;
  static FsFile logFile;
    
public:
  static CircularBuffer<LogEntry, MAX_BUF> _logBuffer;
  static bool begin();
  static void sync();
  static void log(LogLevel level, const char* module, const char* msg);
  static void process();
  static const char* levelToStr(LogLevel level);
};