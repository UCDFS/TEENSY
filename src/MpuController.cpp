#include "MpuController.h"

MpuController::MpuController(Adafruit_MPU6050 mpu) : 
  _mpu(mpu), _mpuFound(false), 
  _gyroOffsetX(0), _gyroOffsetY(0), _gyroOffsetZ(0), 
  _accelOffsetX(0), _accelOffsetY(0), _accelOffsetZ(0) {}

bool MpuController::calibrate() {
  if (!_mpuFound) {
    return false;
  }

  // TODO: print nextion calibrating, progress

  sensors_event_t a, g, t;

  for (uint8_t i = 0; i < 100; i++) {
    this->_mpu.getEvent(&a, &g, &t);
    _gyroOffsetX += g.gyro.x;
    _gyroOffsetY += g.gyro.y;
    _gyroOffsetZ += g.gyro.z;
    _accelOffsetX += a.acceleration.x;
    _accelOffsetY += a.acceleration.y;
    _accelOffsetZ += a.acceleration.z;
    delay(10);
  }

  _gyroOffsetX /= 100.0;
  _gyroOffsetY /= 100.0;
  _gyroOffsetZ /= 100.0;
  _accelOffsetX /= 100.0;
  _accelOffsetY /= 100.0;
  _accelOffsetZ /= 100.0;

  return true;
}

bool MpuController::begin() {
  for (uint8_t i = 0; i < 3; i++) {
    _mpuFound = _mpu.begin();
    if (_mpuFound) {
      break;
    }
    delay(100);
  }
  if (!_mpuFound) {
    // TODO: print nextion MPU not found after 3 tries, continuing without");
    Logger::log(LogLevel::ERROR, "IMU", "MPU not found after 3 tries, continuing without gyro telemetry");
    return false;
  }
  
  _mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
  _mpu.setGyroRange(MPU_GYRO_RANGE);
  _mpu.setFilterBandwidth(MPU_FILTER_BW);

  Nextion::bootStatus("INITIALISATION", "MPU found, starting calibration");
  Logger::log(LogLevel::INFO, "IMU", "MPU found, starting calibration");
  if (calibrate()) {
    Nextion::bootStatus("INITIALISATION", "MPU Calibration successful");
    Logger::log(LogLevel::INFO, "IMU", "Calibration successful");
  } else {
    Nextion::bootStatus("INITIALISATION", "MPU Calibration failed");
    Logger::log(LogLevel::ERROR, "IMU", "Calibration failed");
  }

  return true;
}

// Reads accel+gyro, subtracts boot offsets, then feeds both the decel-light
// trigger and the calibration wizard. Call every loop - I2C read is a single
// fast transaction, no throttling needed.
void MpuController::read() {
  if (!_mpuFound) return;

  sensors_event_t a, g, t;
  _mpu.getEvent(&a, &g, &t);
  ax = a.acceleration.x - _accelOffsetX;
  ay = a.acceleration.y - _accelOffsetY;
  az = a.acceleration.z - _accelOffsetZ;
  gx = g.gyro.x - _gyroOffsetX;
  gy = g.gyro.y - _gyroOffsetY;
  gz = g.gyro.z - _gyroOffsetZ;

  updateDecel();

  switch (_calState) {
    case ImuCalState::IDLE:
    case ImuCalState::DONE:
      break;

    case ImuCalState::BASELINE: {
      _calAccum[0] += ax; _calAccum[1] += ay; _calAccum[2] += az;
      _calSampleCount++;
      if (millis() - _calStateStart >= IMU_CAL_BASELINE_MS) {
        _calBaseline[0] = _calAccum[0] / _calSampleCount;
        _calBaseline[1] = _calAccum[1] / _calSampleCount;
        _calBaseline[2] = _calAccum[2] / _calSampleCount;
        _calPeakDelta[0] = _calPeakDelta[1] = _calPeakDelta[2] = 0;
        _calState = ImuCalState::WAIT_TILT;
        _calStateStart = millis();
        Logger::log(LogLevel::INFO, "IMU", "Cal: baseline captured, lift the rear now");
      }
      break;
    }

    case ImuCalState::WAIT_TILT: {
      float cur[3] = {ax, ay, az};
      for (int i = 0; i < 3; i++) {
        float delta = cur[i] - _calBaseline[i];
        if (fabsf(delta) > fabsf(_calPeakDelta[i])) _calPeakDelta[i] = delta;
      }
      if (millis() - _calStateStart >= IMU_CAL_TILT_MS) {
        int best = 0;
        for (int i = 1; i < 3; i++)
          if (fabsf(_calPeakDelta[i]) > fabsf(_calPeakDelta[best])) best = i;
        _calDetectedAxis = best;
        _calDetectedSign = (_calPeakDelta[best] >= 0) ? 1 : -1;
        _calState = ImuCalState::DONE;
        Logger::log(LogLevel::INFO, "IMU", "Cal: done");
      }
      break;
    }
  }
}

// Sustained-past-ON latch (rejects single-sample bump/vibration noise),
// fast release below OFF. IMU_DECEL_AXIS < 0 means uncalibrated: never fires.
void MpuController::updateDecel() {
  if (IMU_DECEL_AXIS < 0 || IMU_DECEL_AXIS > 2) {
    _decelActive = false;
    return;
  }
  float axes[3] = {ax, ay, az};
  float signedVal = axes[IMU_DECEL_AXIS] * IMU_DECEL_SIGN;

  if (signedVal > IMU_DECEL_ON_MPS2) {
    if (_decelAboveSince == 0) _decelAboveSince = millis();
    if (millis() - _decelAboveSince >= IMU_DECEL_SUSTAIN_MS) _decelActive = true;
  } else {
    _decelAboveSince = 0;
  }
  if (signedVal < IMU_DECEL_OFF_MPS2) {
    _decelActive = false;
  }
}

void MpuController::startCalibration() {
  if (!_mpuFound) return;
  _calState = ImuCalState::BASELINE;
  _calStateStart = millis();
  _calSampleCount = 0;
  _calAccum[0] = _calAccum[1] = _calAccum[2] = 0;
  _calDetectedAxis = -1;
  Logger::log(LogLevel::INFO, "IMU", "Cal: started, hold the car still for baseline");
}

const char *MpuController::calStateName() const {
  switch (_calState) {
    case ImuCalState::IDLE:      return "IDLE";
    case ImuCalState::BASELINE:  return "BASELINE";
    case ImuCalState::WAIT_TILT: return "WAIT_TILT";
    case ImuCalState::DONE:      return "DONE";
  }
  return "IDLE";
}

int MpuController::calProgressPercent() const {
  uint32_t window;
  if (_calState == ImuCalState::BASELINE) window = IMU_CAL_BASELINE_MS;
  else if (_calState == ImuCalState::WAIT_TILT) window = IMU_CAL_TILT_MS;
  else return 0;

  uint32_t elapsed = millis() - _calStateStart;
  int pct = (int)(elapsed * 100 / window);
  return pct > 100 ? 100 : pct;
}

char MpuController::calDetectedAxisName() const {
  if (_calDetectedAxis < 0) return '-';
  return "XYZ"[_calDetectedAxis];
}

float MpuController::calPeakDelta() const {
  if (_calDetectedAxis < 0) return 0;
  return _calPeakDelta[_calDetectedAxis];
}

void MpuController::logTelemetry() {
  if (!_mpuFound) {
    return;
  }

  sensors_event_t a, g, t;
  this->_mpu.getEvent(&a, &g, &t);

  float aX, aY, aZ, gX, gY, gZ, temp;
  aX = a.acceleration.x - _accelOffsetX;
  aY = a.acceleration.y - _accelOffsetY;
  aZ = a.acceleration.z - _accelOffsetZ;
  gX = g.gyro.x - _gyroOffsetX;
  gY = g.gyro.y - _gyroOffsetY;
  gZ = g.gyro.z - _gyroOffsetZ;
  temp = t.temperature;

  const char* log_msg = 
    "Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f | Temp: %.2f";
  char formatted_msg[MAX_LOG_LEN];
  snprintf(formatted_msg, sizeof(formatted_msg), log_msg, 
           aX, aY, aZ, gX, gY, gZ, temp);

  Logger::log(LogLevel::INFO, "IMU", formatted_msg);
}