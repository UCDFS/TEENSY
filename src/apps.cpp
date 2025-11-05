/**
 * @file apps.cpp
 * @brief Handles reading and validating Accelerator Pedal Position Sensors
 * (APPS)
 * @author Shane Whelan (UCD Formula Student)
 * @author Asher Olgeirson (UCD Formula Student)
 * @date 2025/2026
 */

#include "apps.h"
#include <cmath>  // For std::fabs

namespace APPS {

std::pair<float, float> get_raw_values() {
  int apps_1_raw = analogRead(APPS_1_PIN);
  int apps_2_raw = analogRead(APPS_2_PIN);
  return {apps_1_raw, apps_2_raw};
}

std::pair<float, float>
get_percentage_values(std::pair<float, float> raws) {

  // Convert raw ADC values directly to percentage (0-100) - INVERTED values
  // Note: values decrease as pedal is pressed, so we invert the calculation

  // Implement a small deadzone at the top (released) and bottom (fully
  // pressed) so small noise does not produce a non-zero percent. We snap to
  // exact 0% or 100% when within APPS_RAW_DEADZONE counts of the extreme.

  float apps_1_percent;
  if (raws.first >= (APPS1_RAW_MAX - APPS_RAW_DEADZONE)) {
    apps_1_percent = 0.0f;
  } else if (raws.first <= (APPS1_RAW_MIN + APPS_RAW_DEADZONE)) {
    apps_1_percent = 100.0f;
  } else {
    apps_1_percent = 100.0f * (APPS1_RAW_MAX - raws.first) /
                     (APPS1_RAW_MAX - APPS1_RAW_MIN);
  }

  float apps_2_percent;
  if (raws.second >= (APPS2_RAW_MAX - APPS_RAW_DEADZONE)) {
    apps_2_percent = 0.0f;
  } else if (raws.second <= (APPS2_RAW_MIN + APPS_RAW_DEADZONE)) {
    apps_2_percent = 100.0f;
  } else {
    apps_2_percent = 100.0f * (APPS2_RAW_MAX - raws.second) /
                     (APPS2_RAW_MAX - APPS2_RAW_MIN);
  }

  // Clamp values to 0-100 range as a safety net
  apps_1_percent = constrain(apps_1_percent, 0.0f, 100.0f);
  apps_2_percent = constrain(apps_2_percent, 0.0f, 100.0f);

  return {apps_1_percent, apps_2_percent};
}

bool check_plausiblity(std::pair<float, float> percentages) {

  // Check for implausibility (FSUK T11.8.9: deviation > 10%)
  if (std::fabs(percentages.first - percentages.second) >
      APPS_PLAUSIBILITY_THRESHOLD) {
    if (DEBUG_MODE) {
      Serial.print("APPS Implausibility Detected! APPS1: ");
      Serial.print(percentages.first);
      Serial.print("%, APPS2: ");
      Serial.print(percentages.second);
      Serial.println("%");
    }
    return false; // Indicate implausibility
  } else {
    return true;
  }
}

bool check_integrity(std::pair<float, float> raws) {

  // Verify electrical plausiblity as per T11.9.2
  // Allow a small tolerance on the raw min/max to avoid classifying tiny
  // transients or ADC noise as a wiring fault. If values are outside this
  // expanded window then treat as Fault.
  const int apps1_min_allowed = APPS1_RAW_MIN - APPS_RAW_TOLERANCE;
  const int apps1_max_allowed = APPS1_RAW_MAX + APPS_RAW_TOLERANCE;
  const int apps2_min_allowed = APPS2_RAW_MIN - APPS_RAW_TOLERANCE;
  const int apps2_max_allowed = APPS2_RAW_MAX + APPS_RAW_TOLERANCE;

  if (raws.first > apps1_max_allowed || raws.first < apps1_min_allowed ||
      raws.second > apps2_max_allowed || raws.second < apps2_min_allowed) {
    if (DEBUG_MODE) {
      Serial.print("APP Output Exceeds Limits!");
      Serial.print("APPS1 Raw: ");
      Serial.print(raws.first);
      Serial.print(", APPS1 Max: ");
      Serial.print(APPS1_RAW_MAX);
      Serial.print(", APPS1 Min: ");
      Serial.print(APPS1_RAW_MIN);

      Serial.print(", APPS2 Raw: ");
      Serial.print(raws.second);
      Serial.print(", APPS2 Max: ");
      Serial.print(APPS2_RAW_MAX);
      Serial.print(", APPS2 Min: ");
      Serial.println(APPS2_RAW_MIN);
    }
    return false;
    // It should be acceptable to treat this the same as an implausibility as
    // the requirements for implausibilites are more strict than those for SCS
    // faults
  } else {
    return true;
  }
}

/**
 * @brief Reads APPS sensors, checks for plausibility, and returns average pedal
 * position
 */
// TODO: If Apps>= 25 && Brake pedal is pressed for over 500ms, torque needs to
//  be 0nm. Logic should be handled in main.cpp for this.
AppsReading get_apps_reading() {
  auto raws = get_raw_values();
  auto percentages = get_percentage_values(raws);
  bool plausibility = check_plausiblity(percentages);
  bool integrity = check_integrity(raws);
  double average_percent = (percentages.first + percentages.second) / 2.0;

  if (DEBUG_MODE >= 2) {
    static unsigned long last_apps_print = 0;
    if (millis() - last_apps_print > 1000) {
      Serial.print("APPS Readings - Raw: ");
      Serial.print(raws.first);
      Serial.print(", ");
      Serial.print(raws.second);
      Serial.print(" | Percent: ");
      Serial.print(percentages.first, 1);
      Serial.print(", ");
      Serial.print(percentages.second, 1);
      Serial.print(" | Avg: ");
      Serial.println(average_percent, 1);
      last_apps_print = millis();
    }
  }

  if (!integrity) {
    return {AppsStatus::Fault, 0.0, percentages.first,
            percentages.second};
  } else if (!plausibility) {
    return {AppsStatus::Implausible, 0.0, percentages.first,
            percentages.second};
  } else {
    return {AppsStatus::OK, average_percent, percentages.first,
            percentages.second};
  }
}

} // namespace APPS
