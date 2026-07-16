#include "Logger.h"

SdFs Logger::sd;
FsFile Logger::logFile;
FsFile Logger::telemetryFile;
CircularBuffer<LogEntry, MAX_BUF> Logger::_logBuffer;
const char* Logger::_driverName = nullptr;

const char* Logger::levelToStr(LogLevel level) {
    switch (level) {
        case LogLevel::ERROR:   return "ERROR";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::DEBUG:   return "DEBUG";
        default:                return "INFO";
    }
}

void Logger::setDriver(const char* name) {
    _driverName = name;
}

void Logger::log(LogLevel level, const char* module, const char* msg) {
    LogEntry entry;
    if (_driverName != nullptr) {
        snprintf(entry.data, MAX_LOG_LEN, "[%s] [%s] %s: %s", levelToStr(level), _driverName, module, msg);
    } else {
        snprintf(entry.data, MAX_LOG_LEN, "[%s] %s: %s", levelToStr(level), module, msg);
    }
    
    if (SERIAL_LOG_LEVEL >= level) {
        Serial.println(entry.data);
    }
    
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

uint32_t Logger::nextSessionNumber() {
    uint32_t session = 1;

    FsFile f;
    if (f.open("session.cnt", O_RDONLY)) {
        char buf[16] = {0};
        int n = f.read(buf, sizeof(buf) - 1);
        f.close();
        if (n > 0) {
            buf[n] = '\0';
            uint32_t parsed = (uint32_t)atol(buf);
            if (parsed > 0) session = parsed;
        }
    }

    if (f.open("session.cnt", O_RDWR | O_CREAT | O_TRUNC)) {
        char buf[16];
        int n = snprintf(buf, sizeof(buf), "%lu", (unsigned long)(session + 1));
        f.write(buf, n);
        f.sync();
        f.close();
    }

    return session;
}

bool Logger::begin() {
    // SdioConfig(FIFO_SDIO) tells the Teensy to use the fast onboard slot
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        return false;
    }

    uint32_t session = nextSessionNumber();

    // O_RDWR | O_CREAT | O_AT_END are standard POSIX flags. Each session
    // gets its own file - O_AT_END is a no-op on a brand new file but
    // harmless to leave in case a same-numbered file somehow already exists.
    char logName[24];
    snprintf(logName, sizeof(logName), "log_%04lu.txt", (unsigned long)session);
    if (!logFile.open(logName, O_RDWR | O_CREAT | O_AT_END)) {
        return false;
    }

    char telemName[24];
    snprintf(telemName, sizeof(telemName), "telem_%04lu.csv", (unsigned long)session);
    if (!telemetryFile.open(telemName, O_RDWR | O_CREAT | O_AT_END)) {
        return false;
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "Session %lu started", (unsigned long)session);
    log(LogLevel::INFO, "Logger", msg);

    return true;
}

void Logger::process() {
    LogEntry entry;
    // Drain the whole buffer every call - MAX_BUF is now sized to absorb a
    // full CAN burst, so there's no reason to cap this and let entries sit
    // queued across loop iterations (previously capped at 5/call, which is
    // most of why 16 slots overflowed constantly).
    while (_logBuffer.pop(entry)) {
        logFile.println(entry.data);
    }

    uint32_t dropped = _logBuffer.takeDroppedCount();
    if (dropped > 0) {
        char msg[MAX_LOG_LEN];
        snprintf(msg, sizeof(msg), "log buffer overflowed, dropped %lu entries", (unsigned long)dropped);
        log(LogLevel::WARNING, "Logger", msg);  // queued for next process() call
    }

    // Sync every 500ms
    static uint32_t lastSync = 0;
    if (millis() - lastSync > 500) {
        logFile.sync();
        lastSync = millis();
    }
}

void Logger::writeTelemetryHeader(const char *header) {
    telemetryFile.println(header);
    telemetryFile.sync();
}

void Logger::writeTelemetryRow(const char *csvLine) {
    telemetryFile.println(csvLine);

    static uint32_t lastSync = 0;
    if (millis() - lastSync > TELEMETRY_SYNC_MS) {
        telemetryFile.sync();
        lastSync = millis();
    }
}
