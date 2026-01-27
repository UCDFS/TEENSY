/**
 * @file dashboard.h
 * @brief Declares the Nextion dashboard interface for telemetry and sensor pages.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>
#include <Nextion.h>

namespace Dashboard {
  enum class SpeedLimitState { Normal, Tapering, Capped };
  void init(HardwareSerial &serial = Serial1);
  void updateTelemetry(float speed, int rpm, bool rtdActive);
  void updateSensors(float apps1, float apps2,
                     float brakeFront, float brakeRear);
  void updateAcceleratorBar(float accelPercent);
  void updateBrakeLightBar(bool brakeActive);
  // Set speed text color depending on whether speed limiting is active
  void setSpeedLimited(bool limited);
  // New: explicit 3-state color for speed limit status
  void setSpeedLimitState(SpeedLimitState state);
} // namespace Dashboard