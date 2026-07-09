#pragma once
#include <FlexCAN_T4.h>
#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --------- GLOBAL ----------
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
extern uint32_t lastBamocarRx;

// --------- LOGGING ----------
#define SERIAL_LOG_LEVEL LogLevel::DEBUG  // threshold for logging to serial (NONE to disable)
#define LOG_SD false
#define LOG_SERIAL false
#define MAX_BUF 16
#define MAX_LOG_LEN 128
extern FsFile logFile; 

// --------- BUTTONS ----------
// All buttons wire one leg to GND, other to the Teensy pin: use
// INPUT_PULLUP, pressed = LOW. No pull-up resistors on the breakout board.
#define DEBOUNCE_MS 50
#define BUTTON_PIN 2       // RTD button (J12 pin 1, series R + debounce cap on board)
#define AUX_BUTTON1_PIN 7  // repurposed ESP32_TX socket (J8 pin 2) -> RTD speaker
#define AUX_BUTTON2_PIN 8  // repurposed ESP32_RX socket (J8 pin 3) -> precharge

// --------- RTD / DRIVE ENABLE ----------
// Hold RTD with brake held throughout to enable drive; hold (brake not
// required) to disable. Speaker chimes RTD_CHIME_MS on successful enable.
#define RTD_ENABLE_HOLD_MS  2000
#define RTD_DISABLE_HOLD_MS 1000
#define RTD_CHIME_MS        1500

// --------- PRECHARGE ----------
// Output to precharge circuit (J12 pin 3 via series R, pulldown on board
// keeps it disabled through boot). AUX2 press enables (HIGH); AUX2 held
// PRECHARGE_DISABLE_HOLD_MS disables (LOW).
#define PRECHARGE_EN_PIN 3
#define PRECHARGE_DISABLE_HOLD_MS 1000

// --------- RTD SPEAKER ----------
// Output, drives speaker MOSFET gate. HIGH = speaker on.
#define RTD_SPEAKER_EN_PIN 9

// --------- BRAKE ----------
// Brake light follows brake pedal pressure directly (no button). Each
// channel triggers/clears relative to its OWN rest count, not a shared
// absolute count: front and rear rest baselines don't track together
// (front ~401, rear drifted to ~465 as of 2026-07), so a shared absolute
// ON/OFF pair let rear alone false-latch the light permanently once its
// rest rose past the old threshold.
#define BRAKE_FRONT_PIN A2
#define BRAKE_REAR_PIN  A3
#define BRAKE_LIGHT_PIN 6
#define BRAKE_FRONT_REST 401  // calibrate: raw ADC, pedal fully released
#define BRAKE_REAR_REST  465  // calibrate: raw ADC, pedal fully released
#define BRAKE_LIGHT_ON_DELTA  34  // counts above that channel's rest -> light on
#define BRAKE_LIGHT_OFF_DELTA 14  // counts above that channel's rest -> light off

// ---------- APPS (pedal sensor) config ----------
// Set REST to ADC reading with pedal physically released.
// Set FULL to ADC reading at maximum pedal travel.
// Formula handles both rising and falling sensor directions.
#define APPS1_PIN  A0
#define APPS2_PIN  A1
// Calibrated 2026-07-08 on the breakout board (through its 10k/20k divider).
#define APPS1_REST 1727   // calibrate: ADC at physical zero
#define APPS1_FULL 1036   // calibrate: ADC at full pedal
#define APPS2_REST 1757   // calibrate: ADC at physical zero
#define APPS2_FULL 1060   // calibrate: ADC at full pedal
#define PEDAL_DEADBAND_PERCENT 3
#define PEDAL_PLAUSIBILITY_PERCENT 10
#define MAX_ACCEL_PERCENT 100
#define TORQUE_MAX 32767

// ---------- BSPD (brake-throttle plausibility) ----------
// FSAE EV rule: brake pressed + APPS > BSPD_APPS_PERCENT simultaneously ->
// zero torque, latched until APPS < BSPD_RESET_PERCENT regardless of brake.
#define BSPD_APPS_PERCENT   25.0f
#define BSPD_RESET_PERCENT  5.0f

// ---------- Adafruit MPU -----------
#define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG
#define MPU_FILTER_BW MPU6050_BAND_21_HZ

// ---------- IMU decel-based brake light (2nd trigger, ORed with pedal) ----------
// Axis/sign found via the dashboard calibration wizard: lift the rear so the
// nose dips down (same gravity component real braking inertia would inject
// on that axis), read the detected result off the dashboard, set here, reflash.
// -1 = uncalibrated -> decel trigger disabled, pedal-based light still works.
// Calibrated 2026-07-09: lifted the rear, peak +4.0 m/s^2 on Z, well clear of
// the ON/OFF thresholds below.
#define IMU_DECEL_AXIS 2         // 0=X, 1=Y, 2=Z
#define IMU_DECEL_SIGN 1         // +1 or -1
#define IMU_DECEL_ON_MPS2  1.0f  // sustained accel past this on that axis -> light on
#define IMU_DECEL_OFF_MPS2 0.5f  // falls below this -> light off
#define IMU_DECEL_SUSTAIN_MS 100 // must hold past ON threshold this long (reject bump noise)
#define IMU_CAL_BASELINE_MS 1000 // stationary averaging window before the tilt
#define IMU_CAL_TILT_MS     5000 // window to lift the rear and hold the tilt

// ---------- BAMOCAR ----------
#define BAMOCAR_RX_ID 0x201  // Teensy → Bamocar
#define BAMOCAR_TX_ID 0x181  // Bamocar → Teensy
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;