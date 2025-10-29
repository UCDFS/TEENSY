/**
 * @file brake_light.cpp
 * @brief Implementation of brake light control based on brake pressure and deceleration.
 * @author Charlie Zhang (UCD Formula Student)
 * @date 19-Oct-2025
 */
#include "brake_light.h"

// ---- State ----
Adafruit_MPU6050 mpu;
bool  mpuInitialized = false;

int   brakePressureFront = 0;
int   brakePressureRear  = 0;
int   brakePressureCombined = 0;

int   brakeIdleValueFront = 0;
int   brakeIdleValueRear  = 0;
int   brakeIdleValueCombined = 0;
int   dynamicBrakeThreshold  = 0;

bool  brake_light_on = false;

float accelRestX_ms2 = 0.0f, accelRestY_ms2 = 0.0f, accelRestZ_ms2 = 0.0f;
float ax_f_ms2 = 0.0f, ay_f_ms2 = 0.0f, az_f_ms2 = 0.0f;

// ---- Helpers ----
static inline int readAnalogAvg(uint8_t pin, uint8_t n=8) {
  uint32_t s = 0;
  for (uint8_t i=0; i<n; i++) s += analogRead(pin);
  return int(s / n);
}

static inline float rad2deg(float r) { return r * 57.2957795f; }

float brake_compute_forward_tilt_deg(float ay_ms2, float az_ms2) {
  float angle_rad = atan2f(ay_ms2 - accelRestY_ms2, az_ms2 - accelRestZ_ms2);
  return rad2deg(angle_rad);
}

float brake_read_decel_ms2(float ax_ms2) {
  return -(ax_ms2 - accelRestX_ms2);
}

// ---- IMU init + calibration ----
bool brake_initialize_mpu() {
  if (mpuInitialized) return true;
  if (!mpu.begin()) {
    if (DEBUG_MODE) Serial.println("MPU6050 not found!");
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  ax_f_ms2 = a.acceleration.x;
  ay_f_ms2 = a.acceleration.y;
  az_f_ms2 = a.acceleration.z;

  mpuInitialized = true;
  if (DEBUG_MODE) Serial.println("MPU6050 initialized.");
  return true;
}

void brake_calibrate_imu_rest() {
  if (!mpuInitialized) return;
  if (DEBUG_MODE) {
    Serial.print("Calibrating IMU rest (" );
    Serial.print(IMU_CALIB_SAMPLES);
    Serial.println(" samples)...");
  }

  double sx=0, sy=0, sz=0;
  for (uint16_t i=0; i<IMU_CALIB_SAMPLES; i++) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    sx += a.acceleration.x;
    sy += a.acceleration.y;
    sz += a.acceleration.z;
    delayMicroseconds(500);
  }

  accelRestX_ms2 = sx / IMU_CALIB_SAMPLES;
  accelRestY_ms2 = sy / IMU_CALIB_SAMPLES;
  accelRestZ_ms2 = sz / IMU_CALIB_SAMPLES;

  if (DEBUG_MODE) {
    Serial.print("Rest accel = [");
    Serial.print(accelRestX_ms2, 3); Serial.print(", ");
    Serial.print(accelRestY_ms2, 3); Serial.print(", ");
    Serial.print(accelRestZ_ms2, 3); Serial.println("] m/s^2");
  }
}

// ---- Main logic ----
void brake_light_setup() {
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);

  brakePressureFront    = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_FRONT);
  brakePressureRear     = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_REAR);
  brakePressureCombined = (brakePressureFront + brakePressureRear) / 2;

  brakeIdleValueFront    = brakePressureFront;
  brakeIdleValueRear     = brakePressureRear;
  brakeIdleValueCombined = brakePressureCombined;
  dynamicBrakeThreshold  = brakeIdleValueCombined + BRAKE_THRESHOLD_DELTA; // TODO calibrate

  if (DEBUG_MODE) {
    Serial.print("Brake idle C=");
    Serial.print(brakeIdleValueCombined);
    Serial.print("  threshold=");
    Serial.println(dynamicBrakeThreshold);
  }

  if (brake_initialize_mpu()) {
    brake_calibrate_imu_rest();
  }
}

void brake_light_update() {
  // --- read pressures ---
  brakePressureFront    = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_FRONT);
  brakePressureRear     = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_REAR);
  brakePressureCombined = (brakePressureFront + brakePressureRear) / 2;

  // --- IMU read ---
  float decel_ms2 = 0.0f, tilt_deg = 0.0f;
  if (mpuInitialized) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);

    ax_f_ms2 = ACCEL_EWMA_ALPHA*a.acceleration.x + (1.0f-ACCEL_EWMA_ALPHA)*ax_f_ms2;
    ay_f_ms2 = ACCEL_EWMA_ALPHA*a.acceleration.y + (1.0f-ACCEL_EWMA_ALPHA)*ay_f_ms2;
    az_f_ms2 = ACCEL_EWMA_ALPHA*a.acceleration.z + (1.0f-ACCEL_EWMA_ALPHA)*az_f_ms2;

    decel_ms2 = brake_read_decel_ms2(ax_f_ms2);
    tilt_deg  = brake_compute_forward_tilt_deg(ay_f_ms2, az_f_ms2);
  }

  bool want_on = false;

  // Condition 1: pressure threshold
  if (brakePressureCombined > dynamicBrakeThreshold)
    want_on = true;

  // Condition 2: decel threshold
  if (decel_ms2 >= REGEN_DECEL_THRESHOLD)
    want_on = true;

  // Condition 3: forward tilt ≥ 6°
  if (tilt_deg >= FORWARD_TILT_DEG)
    want_on = true;

  // Apply hysteresis
  if (want_on) {
    if (!brake_light_on) {
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      brake_light_on = true;
      if (DEBUG_MODE >= 2) Serial.println("Brake light ON");
    }
  } else {
    bool below_pressure = brakePressureCombined < (dynamicBrakeThreshold - BRAKE_LIGHT_HYSTERESIS);
    bool below_decel    = decel_ms2 < (REGEN_DECEL_THRESHOLD * 0.8f);
    bool below_tilt     = tilt_deg  < (FORWARD_TILT_DEG - 1.0f);
    if (brake_light_on && below_pressure && below_decel && below_tilt) {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
      brake_light_on = false;
      if (DEBUG_MODE >= 2) Serial.println("Brake light OFF");
    }
  }

  if (DEBUG_MODE >= 5) {
    static uint32_t last = 0;
    if (millis() - last > 500) {
      Serial.print("C=");
      Serial.print(brakePressureCombined);
      Serial.print(" thr=");
      Serial.print(dynamicBrakeThreshold);
      Serial.print(" decel=");
      Serial.print(decel_ms2,1);
      Serial.print(" tilt=");
      Serial.print(tilt_deg,1);
      Serial.print(" light=");
      Serial.println(brake_light_on ? "ON" : "OFF");
      last = millis();
    }
  }
}

void brake_recalibrate_idle() {
  brakePressureFront    = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_FRONT);
  brakePressureRear     = readAnalogAvg(BRAKE_PRESSURE_SENSOR_PIN_REAR);
  brakePressureCombined = (brakePressureFront + brakePressureRear) / 2;
  brakeIdleValueCombined = brakePressureCombined;
  dynamicBrakeThreshold  = brakeIdleValueCombined + BRAKE_THRESHOLD_DELTA; // TODO calibrate
  if (DEBUG_MODE) {
    Serial.print("Recalibrated idle. New threshold=");
    Serial.println(dynamicBrakeThreshold);
  }
}
