#pragma once
#include <FlexCAN_T4.h>
#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --------- GLOBAL ----------
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
extern lastBamocarRx;

// --------- LOGGING ----------
#define SERIAL_LOG_LEVEL LogLevel::NONE  // threshold for logging to serial (NONE to disable)
#define LOG_SD false
#define LOG_SERIAL false
#define MAX_BUF 16
#define MAX_LOG_LEN 128
extern FsFile logFile; 

// --------- BUTTON ----------
#define DEBOUNCE_MS 50
#define BUTTON_PIN 2

// ---------- APPS (pedal sensor) config ----------
// Set REST to ADC reading with pedal physically released.
// Set FULL to ADC reading at maximum pedal travel.
// Formula handles both rising and falling sensor directions.
#define APPS1_PIN  A0
#define APPS2_PIN  A1
#define APPS1_REST 2884   // calibrate: ADC at physical zero
#define APPS1_FULL 1835   // calibrate: ADC at full pedal
#define APPS2_REST 2910   // calibrate: ADC at physical zero
#define APPS2_FULL 1845   // calibrate: ADC at full pedal
#define PEDAL_DEADBAND_PERCENT 3
#define PEDAL_PLAUSIBILITY_PERCENT 10
#define MAX_ACCEL_PERCENT 100
#define TORQUE_MAX 32767

// ---------- Adafruit MPU -----------
#define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG
#define MPU_FILTER_BW MPU6050_BAND_21_HZ

// ---------- BAMOCAR ----------
#define BAMOCAR_RX_ID 0x201  // Teensy → Bamocar
#define BAMOCAR_TX_ID 0x181  // Bamocar → Teensy
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;