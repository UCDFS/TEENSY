/**
 * @file main.cpp
 * @brief Formula Student testing ECU integrates APPS, brake, motor controller, and dashboard subsystems.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
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
static bool torqueCutActive = false;

void setup() {
  Serial.begin(115200);
  delay(300);

  if (DEBUG_MODE) Serial.println("\n--- UCDFS Testing ECU Boot ---");

  BrakeLight::init();
  MotorController::init();
  Dashboard::init(Serial1);  // Nextion connected on Serial1
  // Logging::init();

}

void loop() {
  // --- Sensor updates ---
  auto brake = BrakeLight::update();
  auto apps  = APPS::get_apps_reading();

  // --- Safety: APPS + Brake conflict (FSUK T11.8.10) ---
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
  constexpr int16_t kMaxTorqueCounts = 32767; // full-scale Bamocar torque command
  constexpr float   kMaxTorquePercent = 100.0f;

  int16_t torqueCounts = 0;
  if (MotorController::ready() && !MotorController::faulted() && !torqueCutActive) {
    float clampedPercent = constrain(apps.value, 0.0f, kMaxTorquePercent);
    torqueCounts = static_cast<int16_t>(kMaxTorqueCounts * (clampedPercent / kMaxTorquePercent));
  }

  // Always explicitly send the command — ensures inverter sees zero on faults
  MotorController::setTorque(torqueCounts);

  // --- Dashboard update ---
  static uint32_t lastDash = 0;
  if (millis() - lastDash > 100) {
    float speedKmh = Helper::rpm_to_kmh(MotorController::rpm());

    Dashboard::updateTelemetry(speedKmh,
                               MotorController::rpm(),
                               MotorController::ready());

    Dashboard::updateSensors(apps.APPS1,
                             apps.APPS2,
                             brake.front_pressure,
                             brake.rear_pressure);

    Dashboard::updateAcceleratorBar(apps.value);
    Dashboard::updateBrakeLightBar(brake.brake_active);
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
}
