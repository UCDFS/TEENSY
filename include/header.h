/**
 * @file header.h
 * @brief Global configuration constants and pin assignments for the testing ECU.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>

// ====== GLOBAL FLAGS ======
constexpr bool DEBUG_MODE = true;          // Enable Serial debug output

// ====== PIN ASSIGNMENTS ======
constexpr int RTD_BUTTON_PIN  = 22;  // Momentary Ready-to-Drive button
constexpr int BUZZER_PIN      = 23;  // RTD buzzer output

// ====== CAN SETTINGS ======
constexpr unsigned int CAN_BAUD_RATE = 500000;
constexpr unsigned int CAN_TIMEOUT_MS = 500;

// ====== TIMING ======
constexpr unsigned int STATUS_REQ_INTERVAL_MS = 20;
constexpr unsigned int RPM_REQ_INTERVAL_MS    = 20;

// ====== SPEED LIMITER ======
// Top speed limit in km/h and taper band where torque linearly reduces to 0
constexpr float SPEED_LIMIT_KMH         = 50.0f; // adjust as needed
constexpr float SPEED_LIMIT_TAPER_KMH   = 3.0f;  // linear taper band below limit
constexpr float SPEED_LIMIT_HYST_KMH    = 0.5f;  // optional hysteresis for re-enable
// Slew rate to avoid jerky torque changes when limiting (counts per second)
constexpr int   SPEED_SLEW_COUNTS_PER_SEC = 8000; // adjust to taste

// ====== UI / DEBUG ======
constexpr unsigned int DASHBOARD_UPDATE_INTERVAL_MS = 100;  // Nextion refresh cadence
constexpr unsigned int DEBUG_PRINT_INTERVAL_MS      = 1000; // Serial debug cadence
