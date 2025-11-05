/**
 * @file dashboard.cpp
 * @brief Implements Nextion dashboard initialisation and telemetry updates.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#include "dashboard.h"
#include "nextion_ids.h"
#include "dashboard_helpers.h"
#include "nextion_colors.h"

// ---------- Page 2 – Driving Info ----------
static NexPage pageTelemetry(PAGE_TELEMETRY, 0, "page2");
static NexText tSpeed(PAGE_TELEMETRY, 2, KMH_TXT_NAME);
static NexText tRPM(PAGE_TELEMETRY, 3, RPM_TXT_NAME);
static NexText tRTD(PAGE_TELEMETRY, 4, RTDSTATUS_TXT_NAME);

// ---------- Page 4 – Sensor Diagnostics ----------
static NexPage pageSensors(PAGE_SENSORS, 0, "page4");
static NexText tAPPS1(PAGE_SENSORS, 2, APPS1_TXT_NAME);
static NexText tAPPS2(PAGE_SENSORS, 3, APPS2_TXT_NAME);
static NexText tBrakeFront(PAGE_SENSORS, 4, BRAKE_FRONT_TXT_NAME);
static NexText tBrakeRear(PAGE_SENSORS, 5, BRAKE_REAR_TXT_NAME);

// ---------- Page 5 – Signals ----------
static NexPage pageAccel(PAGE_ACCEL, 0, "page5");
static NexProgressBar accelBar(PAGE_ACCEL, 2, ACCEL_BAR_NAME);
static NexProgressBar brakeLight(PAGE_ACCEL, 3, BRAKE_LIGHT);

// ---------- Nextion Globals ----------
static HardwareSerial *dashSerial = nullptr;

// ---------- Implementation ----------
namespace Dashboard {

void init(HardwareSerial &serial) {
  dashSerial = &serial;
  dashSerial->begin(115200);
  nexInit();
  Serial.println("Nextion dashboard initialised.");
}

void updateTelemetry(float speed, int rpm, bool rtdActive) {
  char buf[16];

  setText(tSpeed, speed, 1);
  itoa(rpm, buf, 10);
  setText(tRPM, buf);
  setText(tRTD, rtdActive ? "READY" : "OFF");
}

void updateSensors(float apps1, float apps2,
                   float brakeFront, float brakeRear) {
  setText(tAPPS1, apps1, 1);
  setText(tAPPS2, apps2, 1);
  setText(tBrakeFront, brakeFront, 1);
  setText(tBrakeRear, brakeRear, 1);
}

void updateAcceleratorBar(float accelPercent) {
  setBar(accelBar, (int)accelPercent);
}

void updateBrakeLightBar(bool brakeActive) {
  setBar(brakeLight, brakeActive ? 100 : 0);
}

void setSpeedLimited(bool limited) {
  // Backwards compat: limited=true -> Capped (Blue), false -> Normal (White)
  setSpeedLimitState(limited ? SpeedLimitState::Capped : SpeedLimitState::Normal);
}

void setSpeedLimitState(SpeedLimitState state) {
  uint16_t color = NextionColors::White;
  switch (state) {
    case SpeedLimitState::Normal:   color = NextionColors::UCD_Pink;   break;
    case SpeedLimitState::Tapering: color = NextionColors::Yellow;  break; // beginning to taper
    case SpeedLimitState::Capped:   color = NextionColors::Cyan;    break; // at/over limit
  }
  tSpeed.Set_font_color_pco(color);
}

} // namespace Dashboard
