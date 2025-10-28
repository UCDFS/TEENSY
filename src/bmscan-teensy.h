#pragma once

#include <Arduino.h>
#include "bmscan-teensy-register.h"
// #include "due_can.h"     // or FlexCAN_T4 if you’re using Teensy 4.x
#include <functional>

// -----------------------------
// Make a generic BytesUnion suitable for CAN payloads
// (8 bytes to cover classic CAN max payload).
// -----------------------------
typedef union {
  uint8_t  bytes[8];
  uint16_t words16[4];
  uint32_t words32[2];
} BytesUnion;

// -----------------------------
// Provide a simple CAN_FRAME if one isn't provided by a CAN library.
// -----------------------------
#ifndef CAN_FRAME
typedef struct {
  uint32_t id;
  uint8_t  length;        // DLC (0..8)
  BytesUnion data;        // payload (0..8 bytes used)
  bool     extended;      // whether extended ID
} CAN_FRAME;
#endif

class BMS_data {
public:
  BMS_data(uint32_t canID, const uint8_t *payload, uint8_t length);
  uint8_t getLength() const;
  BytesUnion getData() const;
  uint32_t getMessageID() const;

private:
  BytesUnion data;
  uint8_t dataLength;
  uint32_t messageID;
};

// ----------------------------------------------------------------

class BMSCAN {
public:
  BMSCAN();

  // --- Data Request Methods ---
  bool requestPackVoltage();
  bool requestPackCurrent();
  bool requestStateOfCharge();
  bool requestTemperatures();
  bool requestStatusFlags();

  // --- Getters ---
  float getPackVoltage() const;
  float getPackCurrent() const;
  float getSOC() const;
  float getAvgTemperature() const;
  bool  getFaultStatus() const;

  // --- CAN ID Management ---
  void setRxID(uint16_t rxID);
  void setTxID(uint16_t txID);
  uint16_t getRxID() const;
  uint16_t getTxID() const;

  // --- Incoming CAN Message Handler ---
  void handleIncomingFrame(const CAN_FRAME &msg);

protected:
  bool _sendCAN(const BMS_data &msg);        // sends a CAN frame (stubbed for now)
  void _parseMessage(const CAN_FRAME &msg);

  float _packVoltage = 0.0f;
  float _packCurrent = 0.0f;
  float _soc = 0.0f;
  float _avgTemp = 0.0f;
  bool  _faultActive = false;

  uint16_t _rxID;
  uint16_t _txID;
};
