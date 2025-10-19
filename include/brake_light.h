/**
 * @file brake_light.h
 * @brief Header file for brake light control based on brake pressure and deceleration.
 * @author Charlie Zhang (UCD Formula Student)
 * @date 19-Oct-2025
 */


#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Pin definitions (update if needed) ---
#define BRAKE_PRESSURE_SENSOR_PIN_FRONT A0   // TODO: correct analog pin
#define BRAKE_PRESSURE_SENSOR_PIN_REAR  A1   // TODO: correct analog pin
#define BRAKE_LIGHT_PIN                 5    // TODO: correct output pin

// --- Thresholds (placeholder values, calibrate on car) ---
static constexpr int   BRAKE_THRESHOLD_DELTA  = 10;   // TODO: ADC counts above idle
static constexpr int   BRAKE_LIGHT_HYSTERESIS = 6;    // TODO: counts below threshold to turn off
static constexpr float REGEN_DECEL_THRESHOLD  = 1.0f; // m/s^2
static constexpr float FORWARD_TILT_DEG       = 6.0f; // degrees

// --- IMU calibration ---
static constexpr uint16_t IMU_CALIB_SAMPLES = 1500; // TODO: tune boot time vs stability
static constexpr float     ACCEL_EWMA_ALPHA = 0.2f; // low-pass smoothing factor

#ifndef DEBUG_MODE
#define DEBUG_MODE 1
#endif

// --- Function declarations ---
void brake_light_setup();
void brake_light_update();
void brake_recalibrate_idle();
bool brake_initialize_mpu();
void brake_calibrate_imu_rest();
float brake_compute_forward_tilt_deg(float ay_ms2, float az_ms2);
float brake_read_decel_ms2(float ax_ms2);
