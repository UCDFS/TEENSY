#include "apps.h"
#include "core_pins.h"
#include "elapsedMillis.h"
#include "header.h"
#include "usb_serial.h"

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ; // Wait max 2 seconds
  Serial.println("\n--- UCD FS EV Controller Starting ---");

  // Check if APPS is non-zero on startup as that might indicate somthing wrong
  if (APPS::get_apps_reading() < 0 || APPS::get_apps_reading() > 5) {
    Serial.println("WARNING: APPS non-zero on init");
  }
}

void loop() {

  // Read pedal position
  double pedal_position = APPS::get_apps_reading();

  // pedal_position will be -1.0 during implausiblity or fault
  if (pedal_position < 0.0) {
    if (APPS::implausibility_start == 0) {
      APPS::implausibility_start = millis();
    }
    if (millis() - APPS::implausibility_start >=
        APPS::PLAUSIBILITY_TIMEOUT_MS) {
      // TODO: Disable Torque
    }
  } else {
    APPS::implausibility_start = 0;
    // TODO: Set Torque
  }
  // TODO: Add shutdown condition if pedal_position >= 25.0 and brake pedal is
  // pressed for over 500ms
}
