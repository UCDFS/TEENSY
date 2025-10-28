#include "bmscan-teensy.h"
// #include "can_manager.h"   // Uncomment when you have a can_manager or your CAN API
// #include "header.h"        // Optional DEBUG_MODE define

// --------------------------------------------------------
// BMS_data Constructor / Accessors
// --------------------------------------------------------
BMS_data::BMS_data(uint32_t canID, const uint8_t *payload, uint8_t length) {
  messageID = canID;
  dataLength = (length > 8) ? 8 : length; // clamp to 8
  // zero out bytes first to avoid garbage
  for (uint8_t i = 0; i < 8; ++i) data.bytes[i] = 0;
  for (uint8_t i = 0; i < dataLength; ++i) {
    data.bytes[i] = payload[i];
  }
}

uint8_t BMS_data::getLength() const { return dataLength; }
BytesUnion BMS_data::getData() const { return data; }
uint32_t BMS_data::getMessageID() const { return messageID; }

// --------------------------------------------------------
// BMSCAN Class Implementation
// --------------------------------------------------------
BMSCAN::BMSCAN() {
  _rxID = BMS_RX_ID;
  _txID = BMS_TX_ID;
}

// --- Send CAN frame ---
// NOTE: this is a stub so the code compiles. Replace body with your CAN API call.
// e.g. return can_manager.send_message(frame);
bool BMSCAN::_sendCAN(const BMS_data &msg) {
  (void)msg; // avoid unused param warning
  // TODO: integrate with your CAN manager / library here.
  return false;
}

// --- Request functions (BMS doesn’t always need explicit requests) ---
bool BMSCAN::requestPackVoltage() {
  uint8_t payload[1] = {0x00};
  return _sendCAN(BMS_data(BMS_CAN_ID_PACK_VOLTAGE, payload, 1));
}

// (Add other request wrappers as needed)

// --- Handle incoming CAN frame ---
void BMSCAN::handleIncomingFrame(const CAN_FRAME &msg) {
  // Filter for the BMS ID range (change if necessary)
  if (msg.id >= BMS_CAN_ID_PACK_VOLTAGE && msg.id <= BMS_CAN_ID_CURRENT_LIMITS) {
    _parseMessage(msg);
  }
}

// --- Parse messages from Orion BMS 2 ---
void BMSCAN::_parseMessage(const CAN_FRAME &msg) {
  switch (msg.id) {
    case BMS_CAN_ID_PACK_VOLTAGE: {
      // Example: two-byte unsigned, 0.1 V resolution
      uint16_t raw = (uint16_t)msg.data.bytes[0] | ((uint16_t)msg.data.bytes[1] << 8);
      _packVoltage = (float)raw * 0.1f;
      break;
    }
    case BMS_CAN_ID_PACK_CURRENT: {
      int16_t raw = (int16_t)((uint16_t)msg.data.bytes[0] | ((uint16_t)msg.data.bytes[1] << 8));
      _packCurrent = (float)raw * 0.1f; // sign-aware
      break;
    }
    case BMS_CAN_ID_PACK_SOC: {
      // Example single byte SOC w/ 0.5% resolution (adjust to your BMS docs)
      _soc = (float)msg.data.bytes[0] * 0.5f;
      break;
    }
    case BMS_CAN_ID_TEMPERATURES: {
      // Example: avg temp in byte 4
      _avgTemp = (float)msg.data.bytes[4];
      break;
    }
    case BMS_CAN_ID_STATUS_FLAGS: {
      _faultActive = (msg.data.bytes[0] & 0x01) != 0;
      break;
    }
    default: {
#ifdef DEBUG_MODE
      Serial.print("Unhandled BMS CAN ID: 0x");
      Serial.println(msg.id, HEX);
#endif
      break;
    }
  }
}

// --- Getters ---
float BMSCAN::getPackVoltage() const { return _packVoltage; }
float BMSCAN::getPackCurrent() const { return _packCurrent; }
float BMSCAN::getSOC() const { return _soc; }
float BMSCAN::getAvgTemperature() const { return _avgTemp; }
bool  BMSCAN::getFaultStatus() const { return _faultActive; }

// --- ID setters/getters ---
void BMSCAN::setRxID(uint16_t rxID) { _rxID = rxID; }
void BMSCAN::setTxID(uint16_t txID) { _txID = txID; }
uint16_t BMSCAN::getRxID() const { return _rxID; }
uint16_t BMSCAN::getTxID() const { return _txID; }
