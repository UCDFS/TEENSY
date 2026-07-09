#pragma once
#include "config.h"
#include "Logger.h"
#include "Nextion.h"

enum class ImuCalState { IDLE, BASELINE, WAIT_TILT, DONE };

class MpuController {
private:
  Adafruit_MPU6050 _mpu;
  bool _mpuFound;
  float _gyroOffsetX;
  float _gyroOffsetY;
  float _gyroOffsetZ;
  float _accelOffsetX;
  float _accelOffsetY;
  float _accelOffsetZ;

  bool calibrate();

  // decel-based brake light (config.h IMU_DECEL_AXIS/SIGN)
  bool _decelActive = false;
  uint32_t _decelAboveSince = 0;
  void updateDecel();

  // axis calibration wizard, dashboard-driven
  ImuCalState _calState = ImuCalState::IDLE;
  uint32_t _calStateStart = 0;
  uint16_t _calSampleCount = 0;
  float _calBaseline[3] = {0, 0, 0};
  float _calAccum[3] = {0, 0, 0};
  float _calPeakDelta[3] = {0, 0, 0};
  int8_t _calDetectedAxis = -1;
  int8_t _calDetectedSign = 1;

public:
  MpuController(Adafruit_MPU6050 mpu);
  bool begin();
  void read();  // call every loop - updates ax/ay/az, gx/gy/gz, decel + cal state
  void logTelemetry();

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  bool found() const { return _mpuFound; }
  bool decelBrakeActive() const { return _decelActive; }

  void startCalibration();
  const char *calStateName() const;
  int calProgressPercent() const;
  char calDetectedAxisName() const;
  int calDetectedSign() const { return _calDetectedSign; }
  float calPeakDelta() const;
};
