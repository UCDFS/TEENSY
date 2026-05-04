#include <Arduino.h>
#include "config.h"
#include "Button.h"
#include "MpuController.h"
#include "Nextion.h"

// GLOBALS
Adafruit_MPU6050 MPU;
MpuController mpuController(MPU);
Button driveButton(BUTTON_PIN);
DashStatus dashStatus;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

void setup() {
  Nextion::begin();
  Nextion::bootStatus("INITIALISATION", "System booting up, initialising components");

  Logger::begin();
  if (SERIAL_LOG_LEVEL != LogLevel::NONE) {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("System Ready!");
  }

  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();

  Nextion::bootStatus("INITIALISATION", "Initialisation complete, starting main loop");
  Logger::log(LogLevel::INFO, "Main", "System initialised, starting main loop");

  Nextion::page(NX_PAGE_DRIVE);
}

void loop() {
  mpuController.logTelemetry();
  //TODO: update dashStatus from CAN



  //TODO: Implement brake and APPS signal checking
  /* EV2.3.1 — if brakes are mechanically actuated AND APPS signals >25% pedal travel 
  (or >5kW, whichever is lower) simultaneously for more than 500ms, commanded torque 
  must be 0 Nm.
  EV2.3.2 — once that condition triggers, torque must stay at 0 Nm until APPS drops 
  below 5% and 0 Nm is commanded — regardless of whether brakes are still on.
  */
}
