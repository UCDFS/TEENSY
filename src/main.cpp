#include <Arduino.h>
#include "brake_light.h"

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { } // wait for serial up to 1.5s
  brake_light_setup();
}

void loop() {
  brake_light_update();
}
