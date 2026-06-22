#include <Arduino.h>
#include "config.h"
#include "Logger.h"
#include "bamocar.h"
#include "Button.h"
#include "MpuController.h"
#include "Nextion.h"

#define DRIVE_HOLD_MS      3000
#define CAN_TIMEOUT_MS     2000
#define TEMP_CAN_TIMEOUT_MS 500
#define CAN_READ_DELAY_MS  50

// ---------- Global definitions ----------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
Adafruit_MPU6050 MPU;
MpuController mpuController(MPU);
Button driveButton(BUTTON_PIN);
DashStatus dashStatus;
int8_t currentStep = 0;
int16_t currentTorque = 0;
uint32_t lastTorqueSend = 0;
bool bamocarOnline = false;
int16_t rpmFeedback = 0;
int16_t statusWord = 0;
uint32_t bamocarErrorWord = 0;
int16_t actualCurrent = 0;
float motorTemp = 0.0f;
float inverterTemp = 0.0f;
float dcBusVoltage = 0.0f;
int16_t apps1Raw = 0;
int16_t apps2Raw = 0;
bool pedalFault = false;
bool driveEnabled = false;
uint32_t lastBAMOCARRx = 0;
bool bamocarOffline = false;
bool inErrorState = false;

// ---------- Helpers ----------
static void holdBar(uint32_t elapsed, uint32_t total, char *buf) {
  int filled = (int)(elapsed * 10 / total);
  if (filled > 10) filled = 10;
  buf[0] = '[';
  for (int i = 0; i < 10; i++) buf[i + 1] = (i < filled) ? '#' : ' ';
  buf[11] = ']';
  buf[12] = '\0';
}

static bool pedalAtRest() {
#ifdef APPS1_PIN
  int raw = analogRead(APPS1_PIN);
  #if defined(APPS1_REST) && defined(APPS1_FULL) && defined(PEDAL_DEADBAND_PERCENT)
    float pct = (float)(APPS1_REST - raw) * 100.0f / (float)(APPS1_REST - APPS1_FULL);
    return pct < PEDAL_DEADBAND_PERCENT;
  #else
    return raw == 0;
  #endif
#else
  return true;
#endif
}

// Blocks until button pressed. Keeps CAN drained and sends a heartbeat
// every 500 ms so the BAMOCAR CAN timeout never expires while waiting.
static void waitForButton(const char *prompt) {
  Nextion::bootStatus(prompt, "press to continue");
  uint32_t lastHeartbeat = 0;
  while (!driveButton.isPressed()) {
    CAN::readCanMessages();
    if (millis() - lastHeartbeat > 500) {
      CAN::requestStatusOnce();
      lastHeartbeat = millis();
    }
    delay(10);
  }
}

// Blocks until button is held continuously for DRIVE_HOLD_MS.
// Releasing and re-pressing resets the timer. t_detail shows a progress bar.
static void waitForButtonHeld(const char *prompt) {
  Nextion::bootStatus(prompt, "hold 3s to enable");
  uint32_t lastHeartbeat = 0;
  uint32_t holdStart = 0;
  for (;;) {
    CAN::readCanMessages();
    if (millis() - lastHeartbeat > 500) {
      CAN::requestStatusOnce();
      lastHeartbeat = millis();
    }

    if (digitalRead(BUTTON_PIN) == HIGH) {
      if (holdStart == 0) holdStart = millis();
      uint32_t elapsed = millis() - holdStart;
      if (elapsed >= DRIVE_HOLD_MS) break;
      char bar[13];
      holdBar(elapsed, DRIVE_HOLD_MS, bar);
      Nextion::sendText(NX_BOOT_DETAIL, bar);
    } else {
      if (holdStart != 0) Nextion::sendText(NX_BOOT_DETAIL, "hold 3s to enable");
      holdStart = 0;
    }
    delay(10);
  }
}

// Full re-enable handshake: mirrors startup steps 3-7.
// Blocks until handshake is complete and pedal is released.
static void reenableDriveSequence() {
  Nextion::bootStatus("RE-ENABLE", "clearing errors...");
  CAN::requestStatusCyclic(CAN_TIMEOUT_MS);
  CAN::requestErrorsCyclic(CAN_TIMEOUT_MS);
  CAN::requestSpeedCyclic(CAN_TIMEOUT_MS);
  CAN::requestCurrentCyclic(CAN_TIMEOUT_MS);
  CAN::requestTempsCyclic(TEMP_CAN_TIMEOUT_MS);
  CAN::clearErrors();
  delay(CAN_TIMEOUT_MS * 2);
  CAN::readCanMessages();

  Nextion::bootStatus("RE-ENABLE", "configuring timeout...");
  CAN::configureCanTimeout(CAN_TIMEOUT_MS);
  delay(CAN_TIMEOUT_MS * 2);

  Nextion::bootStatus("RE-ENABLE", "enabling drive...");
  CAN::clearErrors();
  delay(CAN_TIMEOUT_MS);
  CAN::enableDrive();
  CAN::requestStatusOnce();
  delay(CAN_TIMEOUT_MS * 5);
  CAN::sendTorqueCommand(0);

  Nextion::bootStatus("RELEASE PEDAL", "");
  while (!pedalAtRest()) {
    CAN::readCanMessages();
    delay(CAN_READ_DELAY_MS);
  }

  driveEnabled = true;
  Nextion::page(NX_PAGE_DRIVE);
}

// ---------- Step Execution ----------
void executeStep(int step) {
  currentStep = step;
  switch (step) {
    case 1:
      CAN::requestStatusCyclic(100);
      CAN::requestErrorsCyclic(100);
      CAN::requestSpeedCyclic(100);
      CAN::requestCurrentCyclic(100);
      CAN::requestTempsCyclic(500);
      break;
    case 2:
      CAN::requestDCBusOnce();
      break;
    case 3:
      CAN::clearErrors();
      break;
    case 4:
      CAN::configureCanTimeout(2000);
      break;
    case 5:
      CAN::clearErrors();
      delay(100);
      CAN::enableDrive();
      CAN::requestStatusOnce();
      driveEnabled = true;
      break;
    case 6:
      CAN::sendTorqueCommand(0);
      break;
    case 7:
      break;
  }
}

// ---------- Setup ----------
void setup() {
  Nextion::begin();
  Nextion::bootStatus("INITIALISATION", "System booting up, initialising components");

  if (!Logger::begin()) {
    Nextion::bootStatus("LOGGING", "SD init failed");
  }

  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();

  Nextion::bootStatus("INITIALISATION", "Initialisation complete, starting main loop");
  Logger::log(LogLevel::INFO, "Main", "System initialised, starting main loop");

  Can1.begin();
  Can1.setBaudRate(500000);
  analogReadResolution(12);

  delay(800);

  // --- Step 1: press to start, wait for BAMOCAR ---
  waitForButton("PRESS TO START");
  Nextion::bootStatus("WAITING BAMOCAR", "");
  executeStep(1);
  while (!bamocarOnline) {
    CAN::requestStatusOnce();
    CAN::readCanMessages();
    delay(300);
  }
  Nextion::bootStatus("BAMOCAR ONLINE", "");
  delay(400);

  // --- Steps 2-4: DC bus, clear errors, CAN timeout (automatic) ---
  executeStep(2);
  delay(200);
  CAN::readCanMessages();

  executeStep(3);
  delay(200);

  executeStep(4);
  delay(200);

  // --- Steps 5-7: hold 3s to enable drive and enter torque control ---
  waitForButtonHeld("HOLD 3s: ENABLE");
  Nextion::bootStatus("ENABLING DRIVE", "");
  executeStep(5);
  delay(500);
  executeStep(6);
  delay(500);

  // --- Wait for pedal release (automatic) ---
  Nextion::bootStatus("RELEASE PEDAL", "");
  while (!pedalAtRest()) {
    CAN::readCanMessages();
    delay(50);
  }

  // --- Step 7: enter torque control ---
  executeStep(7);
  Nextion::page(NX_PAGE_DRIVE);
}

// ---------- Loop ----------
void loop() {
  CAN::readCanMessages();
  Logger::process();

  if (currentStep == 7) {
    if (bamocarOnline && millis() - lastBAMOCARRx > 500) {
      if (!bamocarOffline) {
        bamocarOffline = true;
        bamocarOnline = false;
        driveEnabled = false;
        CAN::sendTorqueCommand(0);
        currentTorque = 0;
        Nextion::page(NX_PAGE_BOOT);
        Nextion::bootStatus("BAMOCAR OFFLINE", "waiting...");
      }
    } else if (bamocarOffline && bamocarOnline) {
      bamocarOffline = false;
    }

    if (bamocarErrorWord != 0) {
      if (!inErrorState) {
        inErrorState = true;
        driveEnabled = false;
        CAN::sendTorqueCommand(0);
        currentTorque = 0;
        char detail[32];
        CAN::bamocarErrorDescription(bamocarErrorWord, detail, sizeof(detail));
        Nextion::page(NX_PAGE_BOOT);
        Nextion::bootStatus("ERROR", detail);
      }
    } else if (inErrorState) {
      inErrorState = false;
    }

    static uint32_t holdStart = 0;
    if (!driveEnabled) {
      if (digitalRead(BUTTON_PIN) == HIGH) {
        if (holdStart == 0) holdStart = millis();
        uint32_t elapsed = millis() - holdStart;
        if (elapsed >= DRIVE_HOLD_MS) {
          holdStart = 0;
          reenableDriveSequence();
        } else {
          char bar[13];
          holdBar(elapsed, DRIVE_HOLD_MS, bar);
          Nextion::sendText(NX_DRIVE_STATE, bar);
        }
      } else {
        holdStart = 0;
      }
    } else if (driveButton.isPressed()) {
      CAN::disableDrive();
      driveEnabled = false;
      CAN::sendTorqueCommand(0);
      currentTorque = 0;
      Nextion::sendText(NX_DRIVE_STATE, "OFF");
    }

    if (millis() - lastTorqueSend >= 20) {
      CAN::sendTorqueCommand(driveEnabled ? currentTorque : 0);
      lastTorqueSend = millis();
    }

    dashStatus.speed = 0;
    dashStatus.rpm = rpmFeedback;
    dashStatus.torque = currentTorque;
    dashStatus.dcBusV = (int16_t)dcBusVoltage;
    dashStatus.fault = (bamocarErrorWord != 0);
    dashStatus.driveOn = driveEnabled;
    dashStatus.motorTemp = (int16_t)motorTemp;
    dashStatus.inverterTemp = (int16_t)inverterTemp;
    Nextion::updateDash(dashStatus);
  }
}
