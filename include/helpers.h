/**
 * @file helpers.h
 * @brief Utility helpers shared across ECU modules.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>

namespace Helper {
float rpm_to_kmh(float rpm);
}
