#include "apps.h"
#include "core_pins.h"
#include "elapsedMillis.h"
#include "header.h"
#include "usb_serial.h"
#include <Arduino.h>
#include "brake_light.h"

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ; // Wait max 2 seconds
  Serial.println("\n--- UCD FS EV Controller Starting ---");

  // Check if APPS is non-zero on startup as that might indicate somthing wrong
  if (APPS::get_apps_reading().value < 0 ||
      APPS::get_apps_reading().value > 5) {
    Serial.println("WARNING: APPS non-zero on init");
  }
  if (APPS::get_apps_reading().status == APPS::AppsStatus::Fault) {
    Serial.println("ERROR: APPS fault on init");
  }

  if (APPS::get_apps_reading().status == APPS::AppsStatus::Implausible) {
    Serial.println("ERROR: APPS implausiblity on init");
  }
}

void loop() {

  // Read pedal position
  APPS::AppsReading apps_result = APPS::get_apps_reading();

  // pedal_position will be -1.0 during implausiblity or fault
  if (apps_result.status != APPS::AppsStatus::OK) {
    if (APPS::implausibility_start == 0) {
      APPS::implausibility_start = millis();
    }
    if (millis() - APPS::implausibility_start >=
        APPS::PLAUSIBILITY_TIMEOUT_MS) {
          Serial.println("APPS Implausibility Timeout Reached");
      // TODO: Disable Torque
    }
  } else {
    APPS::implausibility_start = 0;
    // TODO: Set Torque
  }
  // TODO: Add shutdown condition if pedal_position >= 25.0 and brake pedal is
  // pressed for over 500ms
}
