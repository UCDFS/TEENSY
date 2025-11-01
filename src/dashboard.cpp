#include "dashboard.h"

// Page 2 – Driving info
static NexPage pageTelemetry(2, 0, "page2");
static NexText tSpeed(2, 2, "t1");
static NexText tRPM(2, 4, "t3");

// Page 3 – Sensor diagnostics
static NexPage pageSensors(3, 0, "page3");
static NexText tAPPS1(3, 2, "t1");
static NexText tAPPS2(3, 4, "t3");
static NexText tBrakeF(3, 6, "t5");
static NexText tBrakeR(3, 8, "t7");

static NexTouch *nex_listen_list[] = { nullptr };
static HardwareSerial *dashSerial = nullptr;

namespace Dashboard {
  void init(HardwareSerial &serial) {
    dashSerial = &serial;
    dashSerial->begin(115200);
    nexInit();
    Serial.println("Nextion dashboard initialised.");
  }

  void updateTelemetry(float speed, int rpm) {
    Serial.println("Updating dashboard telemetry.");  
    char buf[16];
    dtostrf(speed, 0, 1, buf);
    tSpeed.setText(buf);
    itoa(rpm, buf, 10);
    tRPM.setText(buf);
  }

  void updateSensors(float apps1, float apps2,
                     float brakeFront, float brakeRear) {
    Serial.println("Updating dashboard sensors.");
    char buf[16];
    dtostrf(apps1, 0, 1, buf);  tAPPS1.setText(buf);
    dtostrf(apps2, 0, 1, buf);  tAPPS2.setText(buf);
    dtostrf(brakeFront, 0, 1, buf); tBrakeF.setText(buf);
    dtostrf(brakeRear, 0, 1, buf);  tBrakeR.setText(buf);
  }
}
