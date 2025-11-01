/**
 * @file main.cpp
 * @brief Formula Student Testing ECU – integrates APPS, Brake, Motor Controller, and Dashboard.
 * @author
 * @date 2025
 */

#include <Arduino.h>
#include "header.h"
#include "apps.h"
#include "brake_light.h"
#include "motor_controller.h"
#include "dashboard.h"
#include "helpers.h"
#include "logging.h"

// --- Globals ---
static uint32_t lastTorqueCut = 0;
static bool torqueCutActive = false;

void setup() {
  Serial.begin(115200);
  delay(300);

  Logging::init();


  if (DEBUG_MODE) Serial.println("\n--- FS Testing ECU Boot ---");

  brake_light_setup();
  MotorController::init(RTD_BUTTON_PIN, BUZZER_PIN);
  Dashboard::init();
}

void loop() {
  // --- Sensor updates ---
  auto brake = brake_light_update();
  auto apps  = APPS::get_apps_reading();

  // --- Safety: APPS + Brake conflict (FSUK T11.8.10) ---
  // If throttle >25% and brake active >500 ms → torque = 0
  static uint32_t brakeStart = 0;
  if (brake.brake_active && apps.value >= 25.0) {
    if (brakeStart == 0) brakeStart = millis();
    if (millis() - brakeStart > 500) torqueCutActive = true;
  } else {
    brakeStart = 0;
    torqueCutActive = false;
  }

  // --- Motor controller update ---
  MotorController::update();

// --- Torque command with safety redundancy ---
int16_t torqueCounts = 0;
if (MotorController::ready() && !MotorController::faulted() && !torqueCutActive) {
  torqueCounts = map((int)apps.value, 0, 100, 0, 2000);
}

// Always explicitly send the command — ensures inverter sees zero on faults
MotorController::setTorque(torqueCounts);


static uint32_t lastDash = 0;
if (millis() - lastDash > 100) {
  float speedKmh = Helper::rpm_to_kmh(MotorController::rpm());
  Dashboard::updateTelemetry(speedKmh, MotorController::rpm());
  Dashboard::updateSensors(apps.APPS1, apps.APPS2,
                           brake.front_pressure, brake.rear_pressure);
  lastDash = millis();
}



  // --- Periodic debug output ---
  if (DEBUG_MODE) {
    static uint32_t last = 0;
    if (millis() - last > 1000) {
      Serial.print("APPS: ");
      Serial.print(apps.value, 1);
      Serial.print("% | Brake: ");
      Serial.print(brake.brake_active ? "ON" : "OFF");
      Serial.print(" | RTD: ");
      Serial.print(MotorController::ready() ? "YES" : "NO");
      Serial.print(" | Torque cut: ");
      Serial.print(torqueCutActive ? "YES" : "NO");
      Serial.print(" | RPM: ");
      Serial.println(MotorController::rpm());
      last = millis();
    }
  }

  delay(10);
}
