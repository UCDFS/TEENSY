#include "motor.h"
#include "units.h"

Motor::Motor() {
  Serial.begin(USB_DEBUGGING_PORT); // USB debugging
  Serial2.begin(ESP8266_PORT);      // ESP8266 link (TX2/RX2)

  Can1.begin();
  Can1.setBaudRate(BAUDRATE);
  analogReadResolution(12);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card init failed!");
    while (1)
      ;
  }

  String filename = generateFilename();
  logFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("File open failed!");
    while (1)
      ;
  }

  logFile.println("Time(ms),Dir,ID,Len,B0,B1,B2,B3,B4,B5,B6,B7,Decoded");
  logFile.flush();

  Serial.println("=== BAMOCAR Headless Bring-Up ===");
  Serial.print("Logging to: ");
  Serial.println(filename);
  Serial2.println("STATUS:INITIALISING");

  // executeStep(1);

  requestStatusCyclic(100);
  requestSpeedCyclic(100);

  Serial.println("Waiting for Bamocar to respond...");
  while (!bamocarOnline) {
    requestStatusOnce();
    readCanMessages();
    delay(300);
  }

  Serial.println("Bamocar online detected.");
  Serial2.println("STATUS:ONLINE");

  // executeStep(2);

  requestDCBusOnce();
  delay(200);

  // executeStep(3);
  clearErrors();
  delay(200);
  // executeStep(4);
  configureCanTimeout(CAN_TIMEOUT);
  delay(200);
  // executeStep(5);
  clearErrors();
  delay(100);
  enableDrive();
  requestStatusOnce();

  delay(500);
  // executeStep(6);
  sendTorqueCommand(0);
  delay(500);

  Serial.println("Waiting for pedal release...");
  int potValue = 0;
  for (int i = 0; i < 10; i++) {
    potValue += analogRead(A0);
    delay(10);
  }

  potValue /= 10;
  while ((2930 - potValue) * 100.0f / (2930 - 1860) > 5.0f) {
    potValue = analogRead(A0);
    delay(50);
  }
  Serial.println("Pedal released, continuing...");

  // executeStep(7);
  Serial.println("Torque control active (A0)");
  Serial.printf("Max accel cap: %d%%\n", MAX_ACCEL_PERCENT);
  Serial2.println("STATUS:TORQUE_CONTROL");
}

Motor::MotorResponse Motor::set_torque(units::torque::newton_meter_t desired) {
  if (desired < units::torque::newton_meter_t{0}) {
    return MotorResponse::NEGATIVE_TORQUE;
  }
  if (desired > this->MAX_TORQUE) {
    return MotorResponse::OVERTORQUE;
  }
  this->sendTorqueCommand(desired.value());
}

void Motor::transmit(int function, int val, int val2 /*= 0x00*/) {
  CAN_message_t msg = {0};
  msg.id = BAMOCAR_RX_ID;
  msg.len = 3;
  msg.buf[0] = function;
  msg.buf[1] = val;
  msg.buf[2] = val2;
  sendCAN(msg);
}

void Motor::set(int function, int val);
void Motor::request(int field);
