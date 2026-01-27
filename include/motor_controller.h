/**
 * @file motor_controller.h
 * @brief Interface for Bamocar D3 motor controller management over CAN.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>

namespace MotorController {

inline constexpr int DEBUG_MODE = 1; // 0 disables module logging

// Bamocar torque scaling
inline constexpr int16_t MAX_TORQUE_COUNTS  = 32767; // full-scale command
inline constexpr float   MAX_TORQUE_PERCENT = 100.0f;

// Init pins and CAN. rtdButtonPin is active-low. buzzerPin optional (-1 to disable)
void init();

// Call each loop
void update();

// Request torque. Ignored until readyToDrive is true
void setTorque(int16_t torqueCounts);

// Query
bool ready();
bool faulted();
int  rpm();
float dcBus();

}
