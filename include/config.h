#pragma once
#include <FlexCAN_T4.h>
#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// --------- LOGGING ----------
#define LOG_SD true
#define LOG_SERIAL false
#define MAX_BUF 16
#define MAX_LOG_LEN 128
extern FsFile logFile; 

// --------- BUTTON ----------
#define DEBOUNCE_MS 50
#define BUTTON_PIN 2

// ---------- Adafruit MPU -----------
#define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG
#define MPU_FILTER_BW MPU6050_BAND_21_HZ