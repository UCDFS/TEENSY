/**
 * @file brake_light.cpp
 * @brief Implementation of brake light control based on brake pressure and deceleration.
 * @author Charlie Zhang (UCD Formula Student)
 * @date 2025/2026
 */
#include "brake_light.h"

namespace BrakeLight {

// ---- State ----
static Adafruit_MPU6050 mpu;
static bool  mpuInitialized = false;

static int   brakePressureFront = 0;
static int   brakePressureRear  = 0;
static int   brakePressureCombined = 0;

static int   brakeIdleValueFront = 0;
static int   brakeIdleValueRear  = 0;
static int   brakeIdleValueCombined = 0;
static int   dynamicBrakeThreshold  = 0;

static bool  brakeLightOn = false;

static float accelRestX_ms2 = 0.0f, accelRestY_ms2 = 0.0f, accelRestZ_ms2 = 0.0f;
static float ax_f_ms2 = 0.0f, ay_f_ms2 = 0.0f, az_f_ms2 = 0.0f;

// ---- Helpers ----
static inline int readAnalogAvg(uint8_t pin, uint8_t n=8) {
  uint32_t s = 0;
  for (uint8_t i=0; i<n; i++) s += analogRead(pin);
  return int(s / n);
}

static inline float rad2deg(float r) { return r * 57.2957795f; }

float compute_forward_tilt_deg(float ay_ms2, float az_ms2) {
  float angle_rad = atan2f(ay_ms2 - accelRestY_ms2, az_ms2 - accelRestZ_ms2);
  return rad2deg(angle_rad);
}

float read_decel_ms2(float ax_ms2) {
  return -(ax_ms2 - accelRestX_ms2);
}

// ---- IMU init + calibration ----
bool initialize_mpu() {
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

void calibrate_imu_rest() {
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
void setup() {
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

  if (initialize_mpu()) {
    calibrate_imu_rest();
  }
}

BrakeData update() {
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

    decel_ms2 = read_decel_ms2(ax_f_ms2);
    tilt_deg  = compute_forward_tilt_deg(ay_f_ms2, az_f_ms2);
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
    if (!brakeLightOn) {
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      brakeLightOn = true;
      if (DEBUG_MODE >= 2) Serial.println("Brake light ON");
    }
  } else {
    bool below_pressure = brakePressureCombined < (dynamicBrakeThreshold - BRAKE_LIGHT_HYSTERESIS);
    bool below_decel    = decel_ms2 < (REGEN_DECEL_THRESHOLD * 0.8f);
    bool below_tilt     = tilt_deg  < (FORWARD_TILT_DEG - 1.0f);
    if (brakeLightOn && below_pressure && below_decel && below_tilt) {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
      brakeLightOn = false;
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
      Serial.println(brakeLightOn ? "ON" : "OFF");
      last = millis();
    }
  }

  BrakeData d{};
  d.front_pressure    = brakePressureFront;
  d.rear_pressure     = brakePressureRear;
  d.combined_pressure = brakePressureCombined;
  d.brake_active      = brakeLightOn;
  d.decel_ms2        = decel_ms2;
  d.tilt_deg         = tilt_deg;
  return d;
}

void recalibrate_idle() {
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

} // namespace BrakeLight
