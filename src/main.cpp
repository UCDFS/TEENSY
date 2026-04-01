#include <Arduino.h>
#include "config.h"
#include "Button.h"
#include "MpuController.h"

Button button(BUTTON_PIN);
Adafruit_MPU6050 mpu;
MpuController mpuController(mpu);

void setup() {
  // put your setup code here, to run once:


}

void loop() {
  // put your main code here, to run repeatedly:
}
