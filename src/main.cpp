#include "main.h"
#include "motor.h"
#include <Arduino.h>
#include "header.h"
#include "apps.h"
#include "brake_light.h"
#include "motor_controller.h"
#include "dashboard.h"
#include "helpers.h"

// --- Globals ---
static bool torqueCutActive = false;

// Slew state for torque limiting
static int16_t g_lastTorqueCmd = 0;
static uint32_t g_lastTorqueTs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  if (DEBUG_MODE) Serial.println("\n--- UCDFS Testing ECU Boot ---");

  BrakeLight::init();
  MotorController::init();
  Dashboard::init(Serial1);  // Nextion connected on Serial1
  // Logging::init();           // TODO: logging right now causes bottleneck or something
                                //       so gotta figure out why

}

void loop() {
  motor.update();
  // put your main code here, to run repeatedly:
}
