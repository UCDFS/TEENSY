#pragma once
#include "config.h"

class Pedal {
private:
  float appsPercent(int raw, int rest, int full);
  bool fault = false;
  uint8_t plausibilityCount = 0;

  // Dashboard-driven APPS calibration wizard: average raw ADC over a fixed
  // window while the driver holds the pedal at rest or fully pressed, same
  // shape as MpuController's IMU calibration wizard.
  enum class CalPhase { NONE, REST, FULL };
  CalPhase calPhase = CalPhase::NONE;
  uint32_t calStart = 0;
  uint32_t calSum1 = 0, calSum2 = 0;
  uint16_t calSamples = 0;
  void updateCal();

public:
  int16_t apps1Raw = 0;
  int16_t apps2Raw = 0;
  float pct = 0;  // averaged APPS %, updated every read() even during a fault/deadband
  int16_t read();

  void startCalRest();
  void startCalFull();
  const char *calPhaseName() const;
  int calProgressPercent() const;
  bool calRestDone = false;
  bool calFullDone = false;
  int16_t calRest1 = 0, calRest2 = 0, calFull1 = 0, calFull2 = 0;
};