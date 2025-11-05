/**
 * @file nextion_colors.h
 * @brief 16-bit RGB565 color constants for Nextion components (pco/bco values).
 * Values match Nextion HMI color codes.
 */
#pragma once
#include <stdint.h>

namespace NextionColors {
  constexpr uint16_t Black   = 0;
  constexpr uint16_t Blue    = 31;
  constexpr uint16_t Green   = 2016;
  constexpr uint16_t Cyan    = 2047;
  constexpr uint16_t Red     = 63488;
  constexpr uint16_t Magenta = 63519;
  constexpr uint16_t Yellow  = 65504;
  constexpr uint16_t White   = 65535;

  constexpr uint16_t UCD_Pink  = 63504;
}
