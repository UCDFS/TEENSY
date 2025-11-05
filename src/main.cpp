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
  // --- Sensor updates ---
  BrakeLight::BrakeData brake = BrakeLight::update();
  auto apps  = APPS::get_apps_reading();
  Dashboard::SpeedLimitState limitState = Dashboard::SpeedLimitState::Normal;

  // --- Safety: APPS + Brake conflict (FSUK T11.8.10) ---
  static uint32_t brakeStart = 0;
  if (brake.brake_active && apps.value >= APPS::APPS_BRAKE_CONFLICT_THRESHOLD) {
    if (brakeStart == 0) brakeStart = millis();
    if (millis() - brakeStart > APPS::BRAKE_PLAUSIBILITY_TIMEOUT_MS) torqueCutActive = true;
  } else {
    brakeStart = 0;
    torqueCutActive = false;
  }

  // --- Motor controller update ---
  MotorController::update();

  // --- Torque command with safety + speed limiter ---
  constexpr int16_t kMaxTorqueCounts = MotorController::MAX_TORQUE_COUNTS;
  constexpr float   kMaxTorquePercent = MotorController::MAX_TORQUE_PERCENT;

  int16_t torqueCounts = 0;
  if (MotorController::ready() && !MotorController::faulted() && !torqueCutActive) {
    // Base torque from APPS
    float clampedPercent = constrain(apps.value, 0.0f, kMaxTorquePercent);

    // Compute speed and limiter scalar
    const int rpmNow = MotorController::rpm();
    const float speedKmh = Helper::rpm_to_kmh(rpmNow);

    float limiter = 1.0f;
    const float limit = SPEED_LIMIT_KMH;
    const float taper = SPEED_LIMIT_TAPER_KMH;

    // Hysteresis for the hard cap region
    static bool speedCapActive = false;
    if (!speedCapActive) {
      if (speedKmh >= limit) speedCapActive = true;
    } else {
      if (speedKmh <= (limit - SPEED_LIMIT_HYST_KMH)) speedCapActive = false;
    }

    if (speedCapActive) {
      limiter = 0.0f; // hold zero torque until we are below limit - hysteresis
    } else if (speedKmh >= (limit - taper)) {
      // Linear taper as we approach the limit from below
      const float within = speedKmh - (limit - taper);
      limiter = constrain(1.0f - (within / taper), 0.0f, 1.0f);
    }

    float limitedPercent = clampedPercent * limiter;
    // Determine limit state for UI coloring
    if (speedCapActive) {
      limitState = Dashboard::SpeedLimitState::Capped;
    } else if (limiter < 0.999f) {
      limitState = Dashboard::SpeedLimitState::Tapering;
    } else {
      limitState = Dashboard::SpeedLimitState::Normal;
    }

    // Convert to counts
    int16_t desiredCounts = static_cast<int16_t>(kMaxTorqueCounts * (limitedPercent / kMaxTorquePercent));

    // Slew-rate limit to avoid jerk (except when forcing to zero)
    const uint32_t now = millis();
    if (g_lastTorqueTs == 0) g_lastTorqueTs = now;
    const uint32_t dtMs = now - g_lastTorqueTs;

    if (desiredCounts == 0 || desiredCounts < g_lastTorqueCmd) {
      // Allow instant reduction towards zero for safety/limit compliance
      torqueCounts = desiredCounts;
    } else {
      // Limit positive rise rate
      const int32_t maxStep = (int32_t)SPEED_SLEW_COUNTS_PER_SEC * (int32_t)dtMs / 1000;
      const int32_t step = desiredCounts - g_lastTorqueCmd;
      if (step > maxStep) torqueCounts = g_lastTorqueCmd + (int16_t)maxStep;
      else torqueCounts = desiredCounts;
    }

    g_lastTorqueCmd = torqueCounts;
    g_lastTorqueTs = now;
  } else {
    // Not ready/fault/cut — ensure zero and reset slew baseline
    torqueCounts = 0;
    g_lastTorqueCmd = 0;
    g_lastTorqueTs = millis();
    limitState = Dashboard::SpeedLimitState::Normal;
  }

  // Always explicitly send the command — ensures inverter sees zero on faults
  MotorController::setTorque(torqueCounts);

  // --- Dashboard update ---
  static uint32_t lastDash = 0;
  if (millis() - lastDash > DASHBOARD_UPDATE_INTERVAL_MS) {
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
    Dashboard::setSpeedLimitState(limitState);
    lastDash = millis();
  }

  // --- Periodic debug output ---
  if (DEBUG_MODE) {
    static uint32_t last = 0;
    if (millis() - last > DEBUG_PRINT_INTERVAL_MS) {
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
