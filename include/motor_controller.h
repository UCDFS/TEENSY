#pragma once
#include <Arduino.h>

namespace MotorController {

// Simple brake data contract from your brake module
struct BrakeData {
  float front_pressure;
  float rear_pressure;
  bool  brake_active;
};

// Init pins and CAN. rtdButtonPin is active-low. buzzerPin optional (-1 to disable)
void init(int rtdButtonPin, int buzzerPin = -1);

// Provide a function that returns current brake data
void setBrakeProvider(BrakeData (*provider)());

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
