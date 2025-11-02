/**
 * @file brake_light.h
 * @brief Header file for brake light control based on brake pressure and deceleration.
 * @author Charlie Zhang (UCD Formula Student)
 * @date 2025/2026
 */


#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

namespace BrakeLight {

inline constexpr int DEBUG_MODE = 1; // 0: off, higher values increase verbosity

// --- Pin definitions (update if needed) ---
constexpr int BRAKE_PRESSURE_SENSOR_PIN_FRONT = A0;   // TODO: correct analog pin
constexpr int BRAKE_PRESSURE_SENSOR_PIN_REAR  = A1;   // TODO: correct analog pin
constexpr int BRAKE_LIGHT_PIN                 = LED_BUILTIN;    // TODO: correct output pin

// --- Thresholds (placeholder values, calibrate on car) ---
static constexpr int   BRAKE_THRESHOLD_DELTA  = 10;   // TODO: ADC counts above idle
static constexpr int   BRAKE_LIGHT_HYSTERESIS = 6;    // TODO: counts below threshold to turn off
static constexpr float REGEN_DECEL_THRESHOLD  = 1.0f; // m/s^2
static constexpr float FORWARD_TILT_DEG       = 6.0f; // degrees

// --- IMU calibration ---
static constexpr uint16_t IMU_CALIB_SAMPLES = 1500; // TODO: tune boot time vs stability
static constexpr float     ACCEL_EWMA_ALPHA = 0.2f; // low-pass smoothing factor

struct BrakeData {
  int front_pressure;
  int rear_pressure;
  int combined_pressure;
  bool brake_active;   // true when brake light logic is ON
  float decel_ms2;
  float tilt_deg;
};

// --- Function declarations ---
void init();
BrakeData update();   // returns current readings
void recalibrate_idle();
bool initialize_mpu();
void calibrate_imu_rest();
float compute_forward_tilt_deg(float ay_ms2, float az_ms2);
float read_decel_ms2(float ax_ms2);

// Query current brake light state (true when logic is ON)
bool is_active();

} // namespace BrakeLight
