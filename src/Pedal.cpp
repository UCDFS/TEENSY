#include "pedal.h"

float Pedal::appsPercent(int raw, int rest, int full) {
  float pct = (float)(rest - raw) * 100.0f / (float)(rest - full);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

int16_t Pedal::read() {
  apps1Raw = analogRead(APPS1_PIN);
  apps2Raw = analogRead(APPS2_PIN);

  float pct1 = appsPercent(apps1Raw, APPS1_REST, APPS1_FULL);
  float pct2 = appsPercent(apps2Raw, APPS2_REST, APPS2_FULL);

  // Plausibility: sensors must agree within PEDAL_PLAUSIBILITY_PERCENT.
  // Fault only latches after 3 consecutive bad readings (~60ms) to reject
  // single noisy samples. Zero torque immediately on any bad reading.
  if (fabsf(pct1 - pct2) > PEDAL_PLAUSIBILITY_PERCENT) {
    plausibilityCount++;
    if (plausibilityCount >= 3) fault = true;
    return 0;
  }

  plausibilityCount = 0;

  // reset fault when pedal is released
  if (fault) {
    if (pct1 < PEDAL_DEADBAND_PERCENT && pct2 < PEDAL_DEADBAND_PERCENT) {
      fault = false;
    } else {
      return 0;
    }
  }

  float pct = (pct1 + pct2) * 0.5f;
  
  if (pct < PEDAL_DEADBAND_PERCENT) {
    pct = 0.0f;
  }
  if (pct > MAX_ACCEL_PERCENT) {
    pct = (float)MAX_ACCEL_PERCENT;
  }

  return (int16_t)(TORQUE_MAX * (pct / 100.0f));
}
