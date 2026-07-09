#include "Pedal.h"

float Pedal::appsPercent(int raw, int rest, int full) {
  float pct = (float)(rest - raw) * 100.0f / (float)(rest - full);
  pct = std::clamp(pct, 0.0f, 100.0f);
  return pct;
}

int16_t Pedal::read() {
  apps1Raw = analogRead(APPS1_PIN);
  apps2Raw = analogRead(APPS2_PIN);

  float pct1 = appsPercent(apps1Raw, APPS1_REST, APPS1_FULL);
  float pct2 = appsPercent(apps2Raw, APPS2_REST, APPS2_FULL);
  pct = (pct1 + pct2) * 0.5f;  // updated even on a fault/deadband return - BSPD needs this live

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

  // Below the deadband: zero torque, not a floored-up minimum - the old
  // std::max(pct, DEADBAND) floored idle pedal (pct=0) up to a 3% torque
  // command, i.e. permanent creep torque at rest.
  if (pct < PEDAL_DEADBAND_PERCENT) return 0;

  // Remap deadband..100 -> 0..100 so there's no torque step at the deadband
  // edge (tip-in should ramp from zero, not jump straight to 3%).
  float scaled = (pct - PEDAL_DEADBAND_PERCENT) * 100.0f / (100.0f - PEDAL_DEADBAND_PERCENT);
  scaled = std::min(scaled, (float)MAX_ACCEL_PERCENT);

  return (int16_t)(TORQUE_MAX * (scaled / 100.0f));
}
