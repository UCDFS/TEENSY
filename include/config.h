#pragma once
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// --------- LOGGING ----------
#define LOG_SD true
#define LOG_SERIAL false
#define MAX_BUF 16
#define MAX_LOG_LEN 128
extern FsFile logFile; 