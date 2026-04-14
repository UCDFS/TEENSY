#include "config.h"
#include "Logger.h"
#include "Nextion.h"

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
public:
  MpuController(Adafruit_MPU6050 mpu);
  bool begin();
  void logTelemetry();
};