#pragma once

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// BMSManager
// BMS-agnostic single source of truth for all BMS interactions.
//
// Goals:
// - Internal storage of pack-level state in a single struct
// - Public getters only (no direct access to state elsewhere)
// - Placeholder CAN receive hook (no CAN library dependency)
// - Clear separation:
//     (1) raw CAN reception -> onCANFrame(...)
//     (2) raw decoding -> decodeFrame(...)
//     (3) interpreted vehicle-safe values -> getters
// -----------------------------------------------------------------------------

struct BMSState {
  bool     online;
  bool     faultActive;

  float    packVoltage;
  float    packCurrent;
  float    stateOfCharge;

  float    minCellVoltage;
  float    maxCellVoltage;
  float    minCellTemp;
  float    maxCellTemp;

  uint32_t faultBitmap;

  // timing/health (optional, but useful for timeouts and diagnostics later on)
  uint32_t lastRxMillis;     // stores a timestamp of the last received CAN frame (for timeout tracking)
};

class BMSManager {
public:
  // Call once at boot
  static void init();

  // Call periodically (e.g. every loop). Updates online status and timeouts.
  static void update();

  // --- getters (vehicle safety interface) ---
  static bool     isOnline();
  static bool     hasFault();

  static float    packVoltage();
  static float    packCurrent();
  static float    soc();

  static float    minCellVoltage();
  static float    maxCellVoltage();
  static float    minCellTemp();
  static float    maxCellTemp();

  static uint32_t faultBitmap();

  // Optional: copy out the current snapshot (read-only)
  static BMSState snapshot();

  // --- CAN hook (BMS-agnostic raw frame entry point) ---
  // CAN driver or CANManager should call this whenever a CAN frame arrives
  // that might belong to the BMS. Filtering rules can be defined later.
  static void onCANFrame(uint32_t id, const uint8_t* data, uint8_t len);

private:
  static BMSState state;

  // ---- configuration knobs (BMS-agnostic defaults) ----
  static constexpr uint32_t OFFLINE_TIMEOUT_MS = 250; // placeholder value

  // ---- internal helpers ----
  static void markRx(); // updates lastRxMillis and online flag
  static void setDefaults();

  // BMS-specific decode logic will be coded here later (empty for now)
  static void decodeFrame(uint32_t id, const uint8_t* data, uint8_t len);

  // Helpers for “raw -> interpreted” separation (empty for now)
  static float clampFloat(float v, float lo, float hi);
};
