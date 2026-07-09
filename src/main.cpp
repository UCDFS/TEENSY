#include <Arduino.h>
#include "config.h"
#include "Logger.h"
#include "bamocar.h"
#include "Button.h"
#include "BrakePedal.h"
#include "Pedal.h"
#include "MpuController.h"
#include "Nextion.h"

#define CAN_TIMEOUT_MS 2000

// ---------- VCU state ----------
// Automotive flow: the Teensy and the BAMOCAR share the same ignition
// switch, so the CAN handshake needs no driver input - it runs
// automatically until the inverter answers, and re-runs by itself if the
// inverter ever drops off the bus. Driver input is only for drive
// enable/disable:
//   WAIT_BAMOCAR: poll status until the inverter responds, then configure
//                 cyclic telemetry + CAN timeout automatically.
//   STANDBY:      inverter online, drive off. Hold brake + RTD for
//                 RTD_ENABLE_HOLD_MS (throttle released) to enter DRIVE.
//   DRIVE:        torque path live. Hold RTD for RTD_DISABLE_HOLD_MS to
//                 drop back to STANDBY.
// Brake light, precharge control and telemetry run in every state, from
// key-on.
enum class VcuState { WAIT_BAMOCAR, STANDBY, DRIVE };
VcuState vcuState = VcuState::WAIT_BAMOCAR;

// ---------- Global definitions ----------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
Adafruit_MPU6050 MPU;
MpuController mpuController(MPU);
Button driveButton(BUTTON_PIN);  // ctor sets INPUT_PULLUP on the RTD pin
Button aux2Button(AUX_BUTTON2_PIN);
BrakePedal brakePedal;
Pedal pedal;
bool bspdFault = false;
DashStatus dashStatus;
bool driveReady = false;  // BAMOCAR handshake + comms config complete
bool brakeLightOn = false;
bool prechargeEnabled = false;
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
bool driveEnabled = false;
uint32_t lastBamocarRx = 0;
bool inErrorState = false;
bool sdOk = false;  // Logger::begin() result at boot - no ongoing SD health check

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
  return pedal.pct < PEDAL_DEADBAND_PERCENT;
}

// BSPD: brake pedal pressed (pressure-based brakeLightOn, not the IMU decel
// trigger) while APPS shows > BSPD_APPS_PERCENT cuts torque and latches until
// APPS drops below BSPD_RESET_PERCENT, independent of brake state - matches
// FSAE EV brake-throttle plausibility rules.
static int16_t applyBspd(int16_t torque, float pedalPct) {
  if (brakeLightOn && pedalPct > BSPD_APPS_PERCENT) bspdFault = true;
  if (bspdFault) {
    if (pedalPct < BSPD_RESET_PERCENT) bspdFault = false;
    else return 0;
  }
  return torque;
}

// One newline-delimited JSON telemetry record with the full live state.
// Self-throttles to 20 Hz so callers can spin as fast as they like.
// `phase` names the current stage (WAIT_BAMOCAR / STANDBY / DRIVE / ERROR).
static void emitTelemetry(const char *phase) {
  static uint32_t lastEmit = 0;
  if (millis() - lastEmit < 50) return;
  lastEmit = millis();

  int a1 = pedal.apps1Raw;  // cached from this loop's pedal.read(), not a fresh sample
  int a2 = pedal.apps2Raw;
  int bf = analogRead(BRAKE_FRONT_PIN);
  int br = analogRead(BRAKE_REAR_PIN);
  float p1 = (float)(APPS1_REST - a1) * 100.0f / (float)(APPS1_REST - APPS1_FULL);
  float p2 = (float)(APPS2_REST - a2) * 100.0f / (float)(APPS2_REST - APPS2_FULL);
  float bfBar = adcToBar(bf), brBar = adcToBar(br);
  float bfPsi = adcToPsi(bf), brPsi = adcToPsi(br);

  char errDesc[24] = "OK";
  if (bamocarErrorWord != 0)
    CAN::bamocarErrorDescription(bamocarErrorWord, errDesc, sizeof(errDesc));

  char line[640];
  snprintf(line, sizeof(line),
    "{\"t\":%lu,\"phase\":\"%s\","
    "\"apps1\":%d,\"apps2\":%d,\"apps1Pct\":%.1f,\"apps2Pct\":%.1f,"
    "\"brakeF\":%d,\"brakeR\":%d,\"brake\":%d,"
    "\"brakeFBar\":%.2f,\"brakeRBar\":%.2f,\"brakeFPsi\":%.1f,\"brakeRPsi\":%.1f,"
    "\"rtd\":%d,\"aux1\":%d,\"aux2\":%d,"
    "\"precharge\":%d,\"driveEn\":%d,\"driveReady\":%d,"
    "\"rpm\":%d,\"torque\":%d,\"torquePct\":%.1f,\"bspdFault\":%d,\"dcBus\":%.1f,"
    "\"motorTemp\":%d,\"invTemp\":%.1f,"
    "\"bamOnline\":%d,\"status\":%d,\"err\":%lu,\"errDesc\":\"%s\","
    "\"imuFound\":%d,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"decelBrake\":%d,"
    "\"imuCal\":{\"state\":\"%s\",\"pct\":%d,\"axis\":\"%c\",\"sign\":%d,\"peak\":%.2f}}",
    (unsigned long)millis(), phase,
    a1, a2, p1, p2,
    bf, br, brakeLightOn ? 1 : 0,
    bfBar, brBar, bfPsi, brPsi,
    digitalRead(BUTTON_PIN) == LOW ? 1 : 0,
    digitalRead(AUX_BUTTON1_PIN) == LOW ? 1 : 0,
    digitalRead(AUX_BUTTON2_PIN) == LOW ? 1 : 0,
    prechargeEnabled ? 1 : 0, driveEnabled ? 1 : 0, driveReady ? 1 : 0,
    rpmFeedback, currentTorque, pedal.pct, bspdFault ? 1 : 0, dcBusVoltage,
    (int)motorTemp, inverterTemp,
    bamocarOnline ? 1 : 0, statusWord,
    (unsigned long)bamocarErrorWord, errDesc,
    mpuController.found() ? 1 : 0, mpuController.ax, mpuController.ay, mpuController.az,
    mpuController.decelBrakeActive() ? 1 : 0,
    mpuController.calStateName(), mpuController.calProgressPercent(),
    mpuController.calDetectedAxisName(), mpuController.calDetectedSign(),
    mpuController.calPeakDelta());
  Serial.println(line);
}

// brakeLightOn is pedal-pressure-only and feeds the RTD enable interlock -
// it must never go true from an IMU bump/vibration spike, or a jolt could
// satisfy "brake held" for drive-enable with the pedal untouched. The IMU
// decel trigger (mpuController.decelBrakeActive()) only ORs into the physical
// lamp output below, never into this flag. Runs every loop, from key-on, in
// every state.
static void updateBrakeLight() {
  brakeLightOn = brakePedal.read();
  bool lampOn = brakeLightOn || mpuController.decelBrakeActive();
  digitalWrite(BRAKE_LIGHT_PIN, lampOn ? HIGH : LOW);
}

// Single-line commands from the dashboard (Web Serial writes), e.g. the IMU
// calibration wizard's "start" button.
static void handleSerialCommand(const char *cmd) {
  if (strcmp(cmd, "CAL_IMU_START") == 0) {
    mpuController.startCalibration();
  }
}

static void pollSerialCommands() {
  static char cmdBuf[16];
  static uint8_t cmdLen = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleSerialCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// Blocking chime on successful RTD enable. One-shot 1.5s at the moment the
// driver enables drive; acceptable freeze since the car is stationary with
// the brake held.
static void chimeSpeaker() {
  digitalWrite(RTD_SPEAKER_EN_PIN, HIGH);
  delay(RTD_CHIME_MS);
  digitalWrite(RTD_SPEAKER_EN_PIN, LOW);
}

// t_systems (Nextion boot page, vscope: global) - one subsystem per row,
// \r is the Nextion line-break byte, not \n. Throttled since it's a status
// board, not live telemetry.
static void updateSystemsPanel() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 500) return;
  lastUpdate = millis();

  // Nextion wants the literal 2-char text "\r" (backslash + r) in the wire
  // bytes - its own command parser expands that into a real line break. A
  // raw 0x0D byte (what C's '\r' collapses to) is NOT the same thing, so
  // this has to be "\\r" in the source to put backslash-r on the wire.
  char buf[128];
  snprintf(buf, sizeof(buf),
    "BAMOCAR: %s\\rIMU: %s\\rPRECHARGE: %s\\rBSPD: %s\\rSD LOG: %s",
    bamocarOnline ? "ONLINE" : "OFFLINE",
    mpuController.found() ? "OK" : "NOT FOUND",
    prechargeEnabled ? "ON" : "OFF",
    bspdFault ? "FAULT" : "OK",
    sdOk ? "OK" : "FAIL");
  Nextion::sendText(NX_BOOT_SYSTEMS, buf);
}

// ---------- BAMOCAR communication ----------
void initCanCommunication() {
  CAN::requestStatusCyclic(100);
  CAN::requestErrorsCyclic(100);
  CAN::requestSpeedCyclic(100);
  CAN::requestCurrentCyclic(100);
  CAN::requestTempsCyclic(500);
}

void enableDriveMode() {
  CAN::clearErrors();
  delay(100);
  CAN::enableDrive();
  CAN::requestStatusOnce();
  driveEnabled = true;
  chimeSpeaker();
}

// ---------- Setup ----------
// Hardware init only - no blocking gates. The BAMOCAR handshake happens
// automatically in loop() (same ignition switch powers both, so the
// inverter shows up by itself).
void setup() {
  // USB serial for telemetry + logging. Do NOT wait on Serial (would hang
  // on the bench with no host attached).
  Serial.begin(115200);

  Nextion::begin();
  Nextion::bootStatus("INITIALISATION", "System booting up, initialising components");

  sdOk = Logger::begin();
  if (!sdOk) {
    Nextion::bootStatus("LOGGING", "SD init failed");
  }

  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();

  Can1.begin();
  Can1.setBaudRate(500000);
  analogReadResolution(12);

  pinMode(AUX_BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(RTD_SPEAKER_EN_PIN, OUTPUT);
  pinMode(PRECHARGE_EN_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(RTD_SPEAKER_EN_PIN, LOW);
  digitalWrite(PRECHARGE_EN_PIN, LOW);

  Nextion::bootStatus("WAITING BAMOCAR", "handshake is automatic");
  Logger::log(LogLevel::INFO, "Main", "Setup complete, waiting for BAMOCAR");
}

// ---------- Loop ----------
void loop() {
  CAN::readCanMessages();
  Logger::process();
  pollSerialCommands();
  mpuController.read();  // must run before updateBrakeLight() - feeds decelBrakeActive()

  // Brake light works from key-on, independent of drive state.
  updateBrakeLight();

  // Accelerator: always read (keeps the plausibility fault-latch tracking
  // live even outside DRIVE), applied to currentTorque further down once
  // brakeLightOn/driveEnabled for this iteration are settled.
  int16_t pedalTorque = pedal.read();

  // --- BAMOCAR rx watchdog: drop everything and re-handshake ---
  if (bamocarOnline && millis() - lastBamocarRx > 500) {
    bamocarOnline = false;
    driveEnabled = false;
    driveReady = false;
    currentTorque = 0;
    vcuState = VcuState::WAIT_BAMOCAR;
    Nextion::page(NX_PAGE_BOOT);
    Nextion::bootStatus("BAMOCAR OFFLINE", "reconnecting...");
    Logger::log(LogLevel::ERROR, "Main", "BAMOCAR rx timeout, reconnecting");
  }

  // --- BAMOCAR error latch ---
  if (bamocarErrorWord != 0) {
    if (!inErrorState) {
      inErrorState = true;
      driveEnabled = false;
      if (vcuState == VcuState::DRIVE) vcuState = VcuState::STANDBY;
      CAN::sendTorqueCommand(0);
      currentTorque = 0;
      char detail[32];
      CAN::bamocarErrorDescription(bamocarErrorWord, detail, sizeof(detail));
      Nextion::page(NX_PAGE_BOOT);
      Nextion::bootStatus("ERROR", detail);
      Logger::log(LogLevel::ERROR, "Main", detail);
    }
  } else if (inErrorState) {
    inErrorState = false;
    if (vcuState != VcuState::WAIT_BAMOCAR) {
      Nextion::page(NX_PAGE_DRIVE);
      Nextion::sendText(NX_DRIVE_STATE, driveEnabled ? "ON" : "OFF");
    }
  }

  // --- State machine ---
  // rtdWaitRelease forces a release between two RTD actions so the same
  // continuous hold that enabled drive cannot also disable it.
  static uint32_t rtdHoldStart = 0;
  static bool rtdHoldAborted = false;
  static bool rtdWaitRelease = false;
  bool rtdDown = digitalRead(BUTTON_PIN) == LOW;
  if (!rtdDown) rtdWaitRelease = false;
  // Debounced press edge, independent of the raw-read hold tracking below -
  // fires exactly once per physical press regardless of how long it's held.
  bool rtdPressed = driveButton.isPressed();

  switch (vcuState) {
    case VcuState::WAIT_BAMOCAR: {
      static uint32_t lastPoll = 0;
      if (millis() - lastPoll >= 300) {
        lastPoll = millis();
        CAN::requestStatusOnce();
      }
      if (bamocarOnline) {
        // One-shot comms configuration (mirrors the old blocking setup).
        // ~600ms of short delays, once per connect - acceptable freeze.
        initCanCommunication();
        CAN::requestDCBusOnce();
        delay(200);
        CAN::readCanMessages();
        CAN::clearErrors();
        delay(200);
        CAN::configureCanTimeout(CAN_TIMEOUT_MS);
        delay(200);
        driveReady = true;
        vcuState = VcuState::STANDBY;
        rtdHoldStart = 0;
        Nextion::page(NX_PAGE_DRIVE);
        Nextion::sendText(NX_DRIVE_STATE, "OFF");
        Logger::log(LogLevel::INFO, "Main", "BAMOCAR online, comms configured");
      }
      break;
    }

    case VcuState::STANDBY: {
      // Quick press (no brake needed) clears BAMOCAR errors immediately.
      // Hold brake + RTD for RTD_ENABLE_HOLD_MS with throttle released to
      // enable drive. Losing the brake or pressing the throttle mid-hold
      // voids the enable attempt until RTD is released and re-pressed.
      if (rtdPressed && !rtdWaitRelease) {
        CAN::clearErrorsSequence();  // lock/unlock cycle - plain clearErrors() can't erase BUS TIMEOUT
        Nextion::sendText(NX_DRIVE_STATE, "CLEARED");
        Logger::log(LogLevel::INFO, "Main", "BAMOCAR errors cleared (RTD press)");
      }

      if (rtdDown && !rtdWaitRelease) {
        if (rtdHoldStart == 0) {
          rtdHoldStart = millis();
          rtdHoldAborted = false;
        }
        // Precharge must have been commanded (AUX2) before drive can enable -
        // this is a "driver requested precharge" gate, not a bus-voltage-
        // confirmed "precharge finished" check (no pack nominal voltage on
        // hand to compare dcBusVoltage against without fabricating one).
        if (!brakeLightOn || !pedalAtRest() || !prechargeEnabled) rtdHoldAborted = true;

        uint32_t elapsed = millis() - rtdHoldStart;
        if (!rtdHoldAborted && elapsed >= RTD_ENABLE_HOLD_MS) {
          rtdHoldStart = 0;
          rtdWaitRelease = true;
          Nextion::sendText(NX_DRIVE_STATE, "ENABLING");
          enableDriveMode();  // clears errors, enables drive, chimes
          CAN::sendTorqueCommand(0);
          vcuState = VcuState::DRIVE;
          Nextion::sendText(NX_DRIVE_STATE, "ON");
          Logger::log(LogLevel::INFO, "Main", "Drive enabled (RTD hold)");
        } else if (!rtdHoldAborted) {
          char bar[13];
          holdBar(elapsed, RTD_ENABLE_HOLD_MS, bar);
          Nextion::sendText(NX_DRIVE_STATE, bar);
        } else {
          Nextion::sendText(NX_DRIVE_STATE, "HOLD BRAKE");
        }
      } else {
        if (rtdHoldStart != 0) Nextion::sendText(NX_DRIVE_STATE, "OFF");
        rtdHoldStart = 0;
        rtdHoldAborted = false;
      }
      break;
    }

    case VcuState::DRIVE: {
      // Hold RTD for RTD_DISABLE_HOLD_MS to disable (brake not required).
      if (rtdDown && !rtdWaitRelease) {
        if (rtdHoldStart == 0) rtdHoldStart = millis();
        if (millis() - rtdHoldStart >= RTD_DISABLE_HOLD_MS) {
          rtdHoldStart = 0;
          rtdWaitRelease = true;
          CAN::disableDrive();
          driveEnabled = false;
          CAN::sendTorqueCommand(0);
          currentTorque = 0;
          vcuState = VcuState::STANDBY;
          Nextion::sendText(NX_DRIVE_STATE, "OFF");
          Logger::log(LogLevel::INFO, "Main", "Drive disabled (RTD hold)");
        }
      } else {
        rtdHoldStart = 0;
      }
      break;
    }
  }

  // AUX1 (pin AUX_BUTTON1_PIN, INPUT_PULLUP set in setup()):
  // reserved for a dashboard function, not yet decided.

  // --- AUX2: press enables precharge, hold PRECHARGE_DISABLE_HOLD_MS
  // disables it ---
  static uint32_t aux2HoldStart = 0;
  if (aux2Button.isPressed() && !prechargeEnabled) {
    prechargeEnabled = true;
    digitalWrite(PRECHARGE_EN_PIN, HIGH);
    Logger::log(LogLevel::INFO, "Main", "Precharge enabled (AUX2 press)");
  }
  if (prechargeEnabled && digitalRead(AUX_BUTTON2_PIN) == LOW) {
    if (aux2HoldStart == 0) aux2HoldStart = millis();
    if (millis() - aux2HoldStart >= PRECHARGE_DISABLE_HOLD_MS) {
      prechargeEnabled = false;
      digitalWrite(PRECHARGE_EN_PIN, LOW);
      aux2HoldStart = 0;
      Logger::log(LogLevel::INFO, "Main", "Precharge disabled (AUX2 held)");
    }
  } else {
    aux2HoldStart = 0;
  }

  // --- Accelerator -> torque, gated by drive state + BSPD ---
  currentTorque = driveEnabled ? applyBspd(pedalTorque, pedal.pct) : 0;

  // --- Torque keep-alive at 50 Hz while the inverter is online ---
  if (bamocarOnline && millis() - lastTorqueSend >= 20) {
    CAN::sendTorqueCommand(driveEnabled ? currentTorque : 0);
    lastTorqueSend = millis();
  }

  // --- Dash + telemetry ---
  dashStatus.speed = 0;
  dashStatus.rpm = rpmFeedback;
  dashStatus.torque = currentTorque;
  dashStatus.dcBusV = (int16_t)dcBusVoltage;
  dashStatus.fault = (bamocarErrorWord != 0);
  dashStatus.driveOn = driveEnabled;
  dashStatus.motorTemp = (int16_t)motorTemp;
  dashStatus.inverterTemp = (int16_t)inverterTemp;
  Nextion::updateDash(dashStatus);
  updateSystemsPanel();

  const char *phase = "STANDBY";
  if (bamocarErrorWord != 0)                   phase = "ERROR";
  else if (vcuState == VcuState::WAIT_BAMOCAR) phase = "WAIT_BAMOCAR";
  else if (vcuState == VcuState::DRIVE)        phase = "DRIVE";
  emitTelemetry(phase);
}
