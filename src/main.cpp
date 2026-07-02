// ============================================================================
//  BAREBONES MOTOR TEST  (branch: barebones-motor)
// ----------------------------------------------------------------------------
//  Minimal, self-contained firmware for a first bench spin of the motor.
//  NOTHING else is connected: no dashboard, no IMU, no SD logging, no second
//  APPS sensor. A single APPS channel (pin 14 / A0) controls torque.
//
//  Serial-driven flow (open the Serial Monitor at 115200 baud):
//    1. LIVE      - streams raw APPS reading @10 Hz. Press any key to calibrate.
//    2. CALIBRATE - "CONFIRM WHEN APPS AT REST" -> key -> samples REST;
//                   "DEPRESS PEDAL"             -> hold + key -> samples FULL.
//    3. ARM       - press key -> Bamocar handshake + enable drive.
//    4. RUN       - single APPS reading -> torque command @50 Hz.
//
//  SAFETY: first-run torque is capped at TORQUE_LIMIT_PERCENT of full scale.
//          Raise deliberately once behaviour is confirmed. Press 'x' in the
//          Serial Monitor at any time during the run to disable drive.
// ============================================================================
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---------- APPS (single channel) ----------
#define APPS_PIN               14         // APPS1 on Teensy 4.1 (pin 14 / A0)
#define ADC_RESOLUTION_BITS    12          // 0..4095
#define PEDAL_DEADBAND_PERCENT 3           // ignore travel below this (noise)

// ---------- Torque ----------
#define TORQUE_FULL_SCALE      32767        // Bamocar max torque command
#define TORQUE_LIMIT_PERCENT   100           // SAFETY CAP for barebones run
#define TORQUE_SEND_INTERVAL_MS 20          // 50 Hz command rate

// ---------- Bamocar CAN protocol (BAMOCAR-PG-D3) ----------
#define BAMOCAR_RX_ID          0x201        // Teensy -> Bamocar
#define BAMOCAR_TX_ID          0x181        // Bamocar -> Teensy
#define REG_TRANSMIT_REQUEST   0x3D
#define REG_STATUS             0x40
#define REG_CLEAR_ERRORS       0x8E
#define REG_CAN_TIMEOUT        0xD0
#define REG_DRIVE_COMMAND      0x51
#define REG_TORQUE_COMMAND     0x90
#define CAN_TIMEOUT_MS         2000

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// Test phases, stepped through from loop().
enum Phase { PHASE_LIVE, PHASE_CALIBRATE, PHASE_RUN };
Phase phase = PHASE_LIVE;

// Calibration captured at runtime (raw ADC counts).
int appsRest = 0;
int appsFull = 0;

bool driveEnabled = false;
uint32_t lastTorqueSend = 0;
uint32_t lastLivePrint = 0;
bool bamocarSeen = false;

// ---------------------------------------------------------------------------
//  CAN helpers
// ---------------------------------------------------------------------------
static void sendReg(uint8_t reg, uint8_t d1, uint8_t d2) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = reg;
  msg.buf[1] = d1;
  msg.buf[2] = d2;
  Can1.write(msg);
}

static void requestStatusOnce()          { sendReg(REG_TRANSMIT_REQUEST, REG_STATUS, 0x00); }
static void clearErrors()                { sendReg(REG_CLEAR_ERRORS, 0x00, 0x00); }
static void configureCanTimeout(uint16_t ms) { sendReg(REG_CAN_TIMEOUT, ms & 0xFF, (ms >> 8) & 0xFF); }
static void sendTorqueCommand(int16_t v) { sendReg(REG_TORQUE_COMMAND, v & 0xFF, (v >> 8) & 0xFF); }

static void enableDrive() {
  sendReg(REG_DRIVE_COMMAND, 0x04, 0x00);   // assert enable bit
  delay(100);
  sendReg(REG_DRIVE_COMMAND, 0x00, 0x00);   // release, drive latched enabled
}

static void disableDrive() { sendReg(REG_DRIVE_COMMAND, 0x04, 0x00); }

// Drain RX so the controller mailbox never backs up. Detect Bamocar presence.
static void drainCan() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.id == BAMOCAR_TX_ID) bamocarSeen = true;
  }
}

// ---------------------------------------------------------------------------
//  Serial keypress helpers
// ---------------------------------------------------------------------------
static void flushSerialInput() { while (Serial.available()) Serial.read(); }

// Blocks until a byte arrives on Serial. Returns that byte.
static char waitForKey() {
  flushSerialInput();
  while (!Serial.available()) delay(5);
  char c = Serial.read();
  flushSerialInput();
  return c;
}

// ---------------------------------------------------------------------------
//  APPS
// ---------------------------------------------------------------------------
// Map raw ADC to 0..100% using runtime calibration. Direction-agnostic
// (works whether the sensor rises or falls with pedal travel).
static float appsPercent(int raw) {
  int span = appsRest - appsFull;
  if (span == 0) return 0.0f;               // uncalibrated / bad cal guard
  float pct = (float)(appsRest - raw) * 100.0f / (float)span;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

// ---------------------------------------------------------------------------
//  Phase handlers
// ---------------------------------------------------------------------------
// Capture REST and FULL end points. Blocking, keypress driven.
static void calibrateApps() {
  Serial.println(F("CONFIRM WHEN APPS AT REST (press any key)..."));
  waitForKey();
  appsRest = analogRead(APPS_PIN);
  Serial.print(F("  REST captured: ")); Serial.println(appsRest);

  Serial.println(F("DEPRESS PEDAL and hold, then press any key..."));
  waitForKey();
  appsFull = analogRead(APPS_PIN);
  Serial.print(F("  FULL captured: ")); Serial.println(appsFull);

  if (abs(appsRest - appsFull) < 100) {
    Serial.println(F("!! REST and FULL too close - bad calibration. Halting."));
    while (true) delay(1000);               // refuse to arm on bad cal
  }
  Serial.println(F("APPS calibrated OK."));
}

// Bamocar handshake then enable drive. Blocking.
static void armDrive() {
  Serial.println(F("Press any key to ARM DRIVE (handshake + enable)..."));
  waitForKey();

  Serial.println(F("Waiting for Bamocar..."));
  while (!bamocarSeen) {
    requestStatusOnce();
    drainCan();
    delay(200);
  }
  Serial.println(F("Bamocar online."));

  clearErrors();               delay(200);
  configureCanTimeout(CAN_TIMEOUT_MS); delay(200);
  clearErrors();               delay(200);
  enableDrive();
  requestStatusOnce();
  delay(500);
  sendTorqueCommand(0);        // command zero before releasing to loop

  driveEnabled = true;
  Serial.print(F("DRIVE ENABLED. Torque capped at "));
  Serial.print(TORQUE_LIMIT_PERCENT);
  Serial.println(F("% of full scale. Press 'x' to disable."));
}

// ---------------------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);                // wait for Serial Monitor
  delay(200);

  analogReadResolution(ADC_RESOLUTION_BITS);

  Can1.begin();
  Can1.setBaudRate(500000);

  Serial.println();
  Serial.println(F("=== BAREBONES MOTOR TEST ==="));
  Serial.println(F("Live APPS reading. Press any key to calibrate."));
}

// ---------------------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------------------
void loop() {
  drainCan();

  switch (phase) {

    // ----- Live raw APPS at 10 Hz until keypress -----
    case PHASE_LIVE:
      if (millis() - lastLivePrint >= 100) {
        lastLivePrint = millis();
        Serial.print(F("APPS raw=")); Serial.println(analogRead(APPS_PIN));
      }
      if (Serial.available()) {
        flushSerialInput();
        phase = PHASE_CALIBRATE;
      }
      break;

    // ----- Calibrate, then arm drive, then run -----
    case PHASE_CALIBRATE:
      calibrateApps();
      armDrive();
      phase = PHASE_RUN;
      break;

    // ----- Torque control from single APPS -----
    case PHASE_RUN:
      // Kill switch from Serial Monitor.
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x' || c == 'X') {
          driveEnabled = false;
          sendTorqueCommand(0);
          disableDrive();
          Serial.println(F("DRIVE DISABLED by keypress."));
        }
      }

      if (millis() - lastTorqueSend >= TORQUE_SEND_INTERVAL_MS) {
        lastTorqueSend = millis();

        int raw = analogRead(APPS_PIN);
        float pct = appsPercent(raw);
        if (pct < PEDAL_DEADBAND_PERCENT) pct = 0.0f;

        int16_t torque = 0;
        if (driveEnabled) {
          float scaled = (float)TORQUE_FULL_SCALE * (TORQUE_LIMIT_PERCENT / 100.0f);
          torque = (int16_t)(scaled * (pct / 100.0f));
        }
        sendTorqueCommand(torque);

        // Light telemetry (~every 500 ms) so the console isn't flooded.
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint >= 500) {
          lastPrint = millis();
          Serial.print(F("raw=")); Serial.print(raw);
          Serial.print(F(" pct=")); Serial.print(pct, 1);
          Serial.print(F(" torque=")); Serial.println(torque);
        }
      }
      break;
  }
}
