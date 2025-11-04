#include "motor_hal.h"
#ifndef INCLUDE_SRC_MOTOR_HAL_CPP_
#define INCLUDE_SRC_MOTOR_HAL_CPP_
// ---------- Logging ----------
String MotorHAL::generateFilename() {
  int index = 1;
  char filename[32];
  do {
    sprintf(filename, "CAN_traffic_logs_%04d.csv", index);
    index++;
  } while (SD.exists(filename));
  return String(filename);
}

void MotorHAL::logCANFrame(const CAN_message_t &msg, const char *dir) {
  logFile.printf("%lu,%s,0x%03X,%d", millis(), dir, msg.id, msg.len);
  if (msg.len > 0)
    logFile.printf(",0x%02X", msg.buf[0]);
  else
    logFile.print(",");
  for (int i = 0; i < 8; i++) {
    if (i < msg.len)
      logFile.printf(",0x%02X", msg.buf[i]);
    else
      logFile.print(",");
  }

  String decoded = interpretBamocarMessage(msg);
  logFile.print("\"");
  for (unsigned int i = 0; i < decoded.length(); i++) {
    char c = decoded[i];
    if (c == '"')
      logFile.print("\"\"");
    else if (c == '\r' || c == '\n')
      logFile.print(' ');
    else
      logFile.print(c);
  }
  logFile.print("\"\r\n");
  logFile.flush();

  // ---------- Stream CAN frame to ESP ----------
  Serial2.printf("CAN:%s,0x%03X,%d", dir, msg.id, msg.len);
  for (int i = 0; i < msg.len; i++)
    Serial2.printf(",0x%02X", msg.buf[i]);
  Serial2.println();
}

void MotorHAL::sendCAN(const CAN_message_t &msg) {
  Can1.write(msg);
  logCANFrame(msg, "TX");
}

// ---------- Setup ----------
void MotorHAL::setup() {
  Serial.begin(115200);  // USB debugging
  Serial2.begin(115200); // ESP8266 link (TX2/RX2)

  Can1.begin();
  Can1.setBaudRate(500000);
  analogReadResolution(12);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card init failed!");
    while (1)
      ;
  }

  String filename = generateFilename();
  logFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("File open failed!");
    while (1)
      ;
  }

  logFile.println("Time(ms),Dir,ID,Len,B0,B1,B2,B3,B4,B5,B6,B7,Decoded");
  logFile.flush();

  Serial.println("=== BAMOCAR Headless Bring-Up ===");
  Serial.print("Logging to: ");
  Serial.println(filename);
  Serial2.println("STATUS:INITIALISING");

  executeStep(1);
  Serial.println("Waiting for Bamocar to respond...");
  while (!bamocarOnline) {
    requestStatusOnce();
    readCanMessages();
    delay(300);
  }

  Serial.println("Bamocar online detected.");
  Serial2.println("STATUS:ONLINE");

  executeStep(2);
  delay(200);
  executeStep(3);
  delay(200);
  executeStep(4);
  delay(200);
  executeStep(5);
  delay(500);
  executeStep(6);
  delay(500);

  Serial.println("Waiting for pedal release...");
  int potValue = 0;
  for (int i = 0; i < 10; i++) {
    potValue += analogRead(A0);
    delay(10);
  }

  potValue /= 10;
  while ((2930 - potValue) * 100.0f / (2930 - 1860) > 5.0f) {
    potValue = analogRead(A0);
    delay(50);
  }
  Serial.println("Pedal released, continuing...");

  executeStep(7);
  Serial2.println("STATUS:TORQUE_CONTROL");
}

// ---------- Loop ----------
void MotorHAL::loop() {
  readCanMessages();

  if (currentStep == 7 && millis() - lastTorqueSend >= 20) {
    updateTorqueFromPot();
    sendTorqueCommand(currentTorque);
    lastTorqueSend = millis();
  }

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 500) {
    logFile.flush();
    lastFlush = millis();
    sendTelemetry();
  }
}

// ---------- Step Execution ----------
void MotorHAL::executeStep(int step) {
  currentStep = step;
  switch (step) {
  case 1:
    requestStatusCyclic(100);
    requestSpeedCyclic(100);
    Serial.println("Step 1: Cyclic STATUS + RPM started.");
    break;
  case 2:
    requestDCBusOnce();
    break;
  case 3:
    clearErrors();
    break;
  case 4:
    configureCanTimeout(2000);
    break;
  case 5:
    clearErrors();
    delay(100);
    enableDrive();
    requestStatusOnce();
    break;
  case 6:
    sendTorqueCommand(0);
    Serial.println("Torque set to 0 for sanity check");
    break;
  case 7:
    Serial.println("Torque control active (A0)");
    Serial.printf("Max accel cap: %d%%\n", MAX_ACCEL_PERCENT);
    break;
  }
}

// ---------- CAN Commands ----------
void MotorHAL::requestStatusCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x3D;
  msg.buf[1] = 0x40;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void MotorHAL::requestStatusOnce() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x3D;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void MotorHAL::requestSpeedCyclic(uint8_t interval_ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x3D;
  msg.buf[1] = 0x30;
  msg.buf[2] = interval_ms;
  sendCAN(msg);
}

void MotorHAL::requestDCBusOnce() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x3D;
  msg.buf[1] = 0xEB;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void MotorHAL::clearErrors() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x8E;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void MotorHAL::configureCanTimeout(uint16_t ms) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0xD0;
  msg.buf[1] = ms & 0xFF;
  msg.buf[2] = (ms >> 8) & 0xFF;
  sendCAN(msg);
}

void MotorHAL::enableDrive() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x51;
  msg.buf[1] = 0x04;
  msg.buf[2] = 0x00;
  sendCAN(msg);
  delay(100);
  msg.buf[1] = 0x00;
  sendCAN(msg);
}

void MotorHAL::disableDrive() {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x51;
  msg.buf[1] = 0x04;
  msg.buf[2] = 0x00;
  sendCAN(msg);
}

void MotorHAL::sendTorqueCommand(int16_t torqueValue) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = 0x90;
  msg.buf[1] = torqueValue & 0xFF;
  msg.buf[2] = (torqueValue >> 8) & 0xFF;
  sendCAN(msg);
}

// ---------- CAN RX ----------
void MotorHAL::readCanMessages() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    logCANFrame(msg, "RX");

    if (msg.id == BAMOCAR_TX_ID && msg.len >= 3) {
      uint8_t reg = msg.buf[0];

      if (reg == 0x40) { // STATUS register
        bamocarOnline = true;
        uint16_t status = msg.buf[1] | (msg.buf[2] << 8);
        bool enabled = status & 0x0001;
        bool ready = status & 0x0004;
        bool fault = status & 0x0040;

        // Debug print to USB Serial
        Serial.printf("→ STATUS 0x%04X | Enabled:%d Ready:%d Fault:%d\n",
                      status, enabled, ready, fault);

        // Send readable status to ESP8266
        Serial2.printf("STATUS:Enabled=%d,Ready=%d,Fault=%d\n", enabled, ready,
                       fault);
      }

      else if (reg == 0x30) { // RPM feedback
        rpmFeedback = msg.buf[1] | (msg.buf[2] << 8);
      }

      else if (reg == 0xEB) { // DC bus voltage
        dcBusVoltage = (msg.buf[1] | (msg.buf[2] << 8)) * 0.1f;
      }
    }
  }
}

// ---------- Interpret known registers ----------
String MotorHAL::interpretBamocarMessage(const CAN_message_t &msg) {
  char buffer[100];
  uint8_t reg = msg.buf[0];

  if (msg.id == BAMOCAR_RX_ID) {
    switch (reg) {
    case 0x3D:
      sprintf(buffer, "Request register 0x%02X", msg.buf[1]);
      break;
    case 0x8E:
      sprintf(buffer, "Clear all error flags");
      break;
    case 0x51:
      if (msg.buf[1] == 0x04)
        sprintf(buffer, "Lock/Disable drive");
      else if (msg.buf[1] == 0x00)
        sprintf(buffer, "Enable drive");
      else
        sprintf(buffer, "Drive control command 0x%02X", msg.buf[1]);
      break;
    case 0x90:
      sprintf(buffer, "Set torque = %d", (msg.buf[1] | (msg.buf[2] << 8)));
      break;
    case 0xD0:
      sprintf(buffer, "Set CAN timeout = %d ms",
              msg.buf[1] | (msg.buf[2] << 8));
      break;
    default:
      sprintf(buffer, "Command 0x%02X sent", reg);
      break;
    }
    return String(buffer);
  }

  if (msg.id == BAMOCAR_TX_ID) {
    switch (reg) {
    case 0x30:
      sprintf(buffer, "Speed feedback = %d rpm",
              msg.buf[1] | (msg.buf[2] << 8));
      break;
    case 0x40:
      sprintf(buffer, "Status word = 0x%04X", msg.buf[1] | (msg.buf[2] << 8));
      break;
    case 0xEB:
      sprintf(buffer, "DC bus voltage = %.1f V",
              (msg.buf[1] | (msg.buf[2] << 8)) * 0.1f);
      break;
    default:
      sprintf(buffer, "Reply register 0x%02X", reg);
      break;
    }
    return String(buffer);
  }
  return "";
}

// ---------- Potentiometer control ----------
void MotorHAL::updateTorqueFromPot() {
  int potValue = analogRead(A0);
  float potPercent = (2930 - potValue) * 100.0f / (2930 - 1860);
  if (potPercent < 0)
    potPercent = 0;
  if (potPercent > MAX_ACCEL_PERCENT)
    potPercent = MAX_ACCEL_PERCENT;
  currentTorque = (int16_t)(TORQUE_MAX * (potPercent / 100.0f));
}

// ---------- Telemetry output ----------
void MotorHAL::sendTelemetry() {
  Serial2.printf("RPM:%d\n", rpmFeedback);
  Serial2.printf("TORQUE:%d\n", currentTorque);
  Serial2.printf("STATUS:%d\n", statusWord);
  Serial2.printf("DCBUS:%.1f\n", dcBusVoltage);
}

#endif // INCLUDE_SRC_MOTOR_HAL_CPP_
