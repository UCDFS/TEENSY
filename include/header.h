/**
 * @file header.h
 * @brief Central header file including libraries, constants, pin definitions,
 * and function prototypes
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */
/**
 * @file header.h
 * @brief Global configuration for the Formula Student testing setup
 */

#pragma once
#include <Arduino.h>

// ====== GLOBAL FLAGS ======
constexpr bool DEBUG_MODE = true;          // Enable Serial debug output
constexpr bool BENCH_TESTING_MODE = true;  // Prevents unsafe vehicle actions

// ====== PIN ASSIGNMENTS ======
constexpr int RTD_BUTTON_PIN  = 22;  // Momentary Ready-to-Drive button
constexpr int BUZZER_PIN      = 23;  // RTD buzzer output

// ====== CAN SETTINGS ======
constexpr unsigned int CAN_BAUD_RATE = 500000;
constexpr unsigned int CAN_TIMEOUT_MS = 500;

// ====== TIMING ======
constexpr unsigned int STATUS_REQ_INTERVAL_MS = 200;
constexpr unsigned int RPM_REQ_INTERVAL_MS    = 200;

// ====== UTILITY ======
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
