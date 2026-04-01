#include "MpuController.h"

MpuController::MpuController(Adafruit_MPU6050 mpu) : 
  _mpu(mpu), _mpuFound(false) {}

bool MpuController::calibrate() {
  if (!_mpuFound) {
    return false;
  }

  sensors_event_t a, g, t;

  for (uint8_t i = 0; i < 100; i++) {
    this->_mpu.getEvent(&a, &g, &t);
    _gyroOffsetX += g.gyro.x;
    _gyroOffsetY += g.gyro.y;
    _gyroOffsetZ += g.gyro.z;
    _accelOffsetX += a.acceleration.x;
    _accelOffsetY += a.acceleration.y;
    _accelOffsetZ += a.acceleration.z;
    _tempOffset += t.temperature;
    delay(10);
  }

  _gyroOffsetX /= 100.0;
  _gyroOffsetY /= 100.0;
  _gyroOffsetZ /= 100.0;
  _accelOffsetX /= 100.0;
  _accelOffsetY /= 100.0;
  _accelOffsetZ /= 100.0;
  _tempOffset /= 100.0;

  return true;
}

bool MpuController::begin() {
  for (uint8_t i = 0; i < 3; i++) {
    _mpuFound = _mpu.begin();
    if (_mpuFound) {
      break;
    }
  }
  if (!_mpuFound) {
    // print nextion MPU not found after 3 tries, continuing without");
    return false;
  }
  
  _mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
  _mpu.setGyroRange(MPU_GYRO_RANGE);
  _mpu.setFilterBandwidth(MPU_FILTER_BW);

  if (calibrate()) {
    // print nextion calibration success
  } else {
    // print nextion calibration failure
  }

  return true;
}

void MpuController::logTelemetry() {
  sensors_event_t a, g, t;
  this->_mpu.getEvent(&a, &g, &t);

  float aX, aY, aZ, gX, gY, gZ, temp;
  aX = a.acceleration.x - _accelOffsetX;
  aY = a.acceleration.y - _accelOffsetY;
  aZ = a.acceleration.z - _accelOffsetZ;
  gX = g.gyro.x - _gyroOffsetX;
  gY = g.gyro.y - _gyroOffsetY;
  gZ = g.gyro.z - _gyroOffsetZ;
  temp = t.temperature - _tempOffset;

  const char* log_msg = 
    "Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f | Temp: %.2f";
  char formatted_msg[MAX_LOG_LEN];
  snprintf(formatted_msg, sizeof(formatted_msg), log_msg, 
           aX, aY, aZ, gX, gY, gZ, temp);

  Logger::log(LogLevel::INFO, "IMU", formatted_msg);
}