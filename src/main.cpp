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

void setup() {
  if (SERIAL_DEBUG) {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("System Ready!");
  }

  Nextion::begin();
  Nextion::bootStatus("INITIALISATION", "System booting up, initialising components");

  Logger::begin();
  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();

  Nextion::bootStatus("INITIALISATION", "Initialisation complete, starting main loop");
  Logger::log(LogLevel::INFO, "Main", "System initialised, starting main loop");

  Nextion::page(NX_PAGE_DRIVE);
}

void loop() {
  mpuController.logTelemetry();
  //TODO: update dashStatus from CAN
}
