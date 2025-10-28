/**
 * @file header.h
 * @brief Central header file including libraries, constants, pin definitions,
 * and function prototypes
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO: calibration below
//   consider using deceleration directly from MPU if required by rules
//   (T6.3.1).
// - Define pins and logic for monitoring critical errors (IMD, BSPD etc.) via
// digital inputs.

#ifndef HEADER_H
#define HEADER_H

// ------------ STANDARD LIBRARIES ------------
#include <cmath>
#include <stdint.h> // For fixed-width integer types

// ------------ TEENSY LIBRARIES ------------

// ------------ CAR LIBRARIES ------------
#include "apps.h"

// Undefine Arduino's min/max macros to avoid conflicts with C++ standard
// library
#undef max
#undef min

#include <limits>

// VCU-HACK: Set to true to bypass BMS checks for bench testing without a BMS.
// TODO: MUST BE FALSE FOR VEHICLE OPERATION.
const bool BENCH_TESTING_MODE = true;
#endif // HEADER_H
