/**
 * @file motor_controller.h
 * @brief Interface for Bamocar D3 motor controller management over CAN.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#pragma once
#include <Arduino.h>
#include "brake_light.h"

namespace MotorController {

using BrakeData = BrakeLight::BrakeData;

inline constexpr int DEBUG_MODE = 1; // 0 disables module logging

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
