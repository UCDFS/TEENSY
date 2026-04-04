#include <Arduino.h>
#include "config.h"
#include "Button.h"
#include "MpuController.h"

Button button(BUTTON_PIN);
Adafruit_MPU6050 mpu;
MpuController mpuController(mpu);

void setup() {
  Logger::begin();
  Logger::log(LogLevel::INFO, "Main", "System booting up, initialising components");

  mpuController.begin();
  Logger::log(LogLevel::INFO, "Main", "MPU initialized successfully");

  Logger::log(LogLevel::INFO, "Main", "System initialized, starting main loop");
}

void loop() {
  mpuController.logTelemetry();
  
}
