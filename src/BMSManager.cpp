#include "BMSManager.h"

// Define static storage
BMSState BMSManager::state;

void BMSManager::init() {
  setDefaults();
}

void BMSManager::setDefaults() {
  state.online        = false;
  state.faultActive   = false;

  state.packVoltage   = 0.0f;
  state.packCurrent   = 0.0f;
  state.stateOfCharge = 0.0f;

  state.minCellVoltage = 0.0f;
  state.maxCellVoltage = 0.0f;
  state.minCellTemp    = 0.0f;
  state.maxCellTemp    = 0.0f;

  state.faultBitmap   = 0;
  state.lastRxMillis  = 0;
}

void BMSManager::update() {
  // Mark as offline if we haven't seen frames recently.
  // This is made intentionally as a placeholder policy and can be edited later.
  if (state.lastRxMillis == 0) {
    state.online = false;
    return;
  }

  const uint32_t now = millis();
  const uint32_t age = now - state.lastRxMillis;

  state.online = (age <= OFFLINE_TIMEOUT_MS);
}

void BMSManager::markRx() {
  state.lastRxMillis = millis();
  state.online = true;
}

void BMSManager::onCANFrame(uint32_t id, const uint8_t* data, uint8_t len) {
  // Basic guards (still BMS-agnostic)
  if (data == nullptr || len == 0) {
    return;
  }

  // Record that we received something that the upper layer (routing/filtering layer) routed here.
  // Shows us the BMS is alive and transmitting something, even if we don't understand the content yet.
  markRx();

  // Decode is intentionally empty/placeholder for now.
  decodeFrame(id, data, len);
}

void BMSManager::decodeFrame(uint32_t id, const uint8_t* data, uint8_t len) {
  (void)id;
  (void)data;
  (void)len;

  // Placeholder: no assumptions about IDs or layout.
  // Can later implement:
  // - match IDs
  // - unpack fields
  // - update state fields
  //
  // IMPORTANT: when implementing this, keep:
  //   - raw decode here
  //   - scaling + safety clamping in a separate helper if needed
}

// --- getters ---
bool BMSManager::isOnline() {
  return state.online;
}

bool BMSManager::hasFault() {
  return state.faultActive;
}

float BMSManager::packVoltage() {
  return state.packVoltage;
}

float BMSManager::packCurrent() {
  return state.packCurrent;
}

float BMSManager::soc() {
  return state.stateOfCharge;
}

float BMSManager::minCellVoltage() {
  return state.minCellVoltage;
}

float BMSManager::maxCellVoltage() {
  return state.maxCellVoltage;
}

float BMSManager::minCellTemp() {
  return state.minCellTemp;
}

float BMSManager::maxCellTemp() {
  return state.maxCellTemp;
}

uint32_t BMSManager::faultBitmap() {
  return state.faultBitmap;
}

BMSState BMSManager::snapshot() {
  return state; // returns a copy
}

float BMSManager::clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
