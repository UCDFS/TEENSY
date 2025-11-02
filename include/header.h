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

