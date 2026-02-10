#include <Arduino.h>
#include "BMSManager.h"

// Added just to prove it links
void setup() {
  BMSManager::init();
}

void loop() {
  BMSManager::update();

  // Other modules should ONLY use getters
  // bool online = BMSManager::isOnline();

  delay(10);
}
