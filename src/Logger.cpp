#include "Logger.h"

SdFs Logger::sd;
FsFile Logger::logFile;
CircularBuffer<LogEntry, MAX_BUF> Logger::_logBuffer;

const char* Logger::levelToStr(LogLevel level) {
    switch (level) {
        case LogLevel::ERROR:   return "ERROR";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::DEBUG:   return "DEBUG";
        default:                return "INFO";
    }
}

void Logger::log(LogLevel level, const char* module, const char* msg) {
    LogEntry entry;
    snprintf(entry.data, MAX_LOG_LEN, "[%s] %s: %s", levelToStr(level), module, msg);
    
    _logBuffer.push(entry);
}

// ---------- CAN frame logging ----------
// Record: C,<ms>,<dir>,<id_hex>,<len>,<b0_hex>,...,<b7_hex>
// Always 13 columns; unused byte fields are empty.
void Logger::logCANFrame(const CAN_message_t &msg, const char *dir) {
  char line[80];
  int n = snprintf(line, 79, "C,%lu,%s,%03lX,%d", millis(), dir, msg.id, msg.len);
  for (int i = 0; i < 8; i++) {
    if (i < msg.len) n += snprintf(line + n, 80 - n, ",%02X", msg.buf[i]);
    else             n += snprintf(line + n, 80 - n, ",");
  }
  line[n++] = '\n';
  log(LogLevel::INFO, "CAN", line);
}

bool Logger::begin() {
    // SdioConfig(FIFO_SDIO) tells the Teensy to use the fast onboard slot
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        return false;
    }

    // O_RDWR | O_CREAT | O_AT_END are standard POSIX flags
    if (!logFile.open("log.txt", O_RDWR | O_CREAT | O_AT_END)) {
        return false;
    }

    return true;
}

void Logger::process() {
    if (_logBuffer.isEmpty()) return;

    LogEntry entry;
    uint8_t count = 0;

    // Pop and write in small bursts
    while (_logBuffer.pop(&entry) && count < 5) {
        logFile.println(entry.data);
        count++;
    }

    // Sync every 500ms
    static uint32_t lastSync = 0;
    if (millis() - lastSync > 500) {
        logFile.sync(); 
        lastSync = millis();
    }
}
