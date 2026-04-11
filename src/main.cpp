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
  Nextion::begin();
  Nextion::bootStatus("INITIALISATION", "System booting up, initialising components");

  Logger::begin();
  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();

  Nextion::bootStatus("INITIALISATION", "Initialisation complete, starting main loop");
  Logger::log(LogLevel::INFO, "Main", "System initialised, starting main loop");
}

void loop() {
  mpuController.logTelemetry();
  //TODO: update dashStatus from CAN
}
