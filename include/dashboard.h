#pragma once
#include <Arduino.h>
#include <Nextion.h>

namespace Dashboard {
  void init(HardwareSerial &serial = Serial1);
  void updateTelemetry(float speed, int rpm);
  void updateSensors(float apps1, float apps2,
                     float brakeFront, float brakeRear);
}
