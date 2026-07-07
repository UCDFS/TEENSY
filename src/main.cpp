// Raw sensor readout firmware
//
// Prints raw ADC counts for APPS1, APPS2, brake front and brake rear
// at 10 Hz over USB serial. No CAN, no dash, no logging - bench tool
// for sensor calibration and wiring checks.
//
// Pins per Teensy 4.1 breakout board netlist:
//   APPS1        pin 14 / A0
//   APPS2        pin 15 / A1
//   Brake front  pin 16 / A2
//   Brake rear   pin 17 / A3

#include <Arduino.h>

#define APPS1_PIN       A0
#define APPS2_PIN       A1
#define BRAKE_FRONT_PIN A2
#define BRAKE_REAR_PIN  A3

#define PRINT_INTERVAL_MS 100  // 10 Hz

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 0-4095, matches existing APPS calibration values
  analogReadAveraging(4);
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = millis();

  char line[80];
  snprintf(line, sizeof(line), "APPS1:%4d  APPS2:%4d  BRAKE_F:%4d  BRAKE_R:%4d",
           analogRead(APPS1_PIN), analogRead(APPS2_PIN),
           analogRead(BRAKE_FRONT_PIN), analogRead(BRAKE_REAR_PIN));
  Serial.println(line);
}
