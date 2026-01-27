#include <Arduino.h>
#include "dashboard.h"
#include "nextion_ids.h"



String sendData;
static HardwareSerial *dashSerial = nullptr;

namespace Dashboard {

void init(HardwareSerial &serial) {
  dashSerial = &serial;
  dashSerial->begin(115200);
  Serial.println("Nextion dashboard initialised.");
}

//kmh,RPMk
//text: .txt
//bar: .val
//position .x, .y



//text
void Send_text(String ID, String Value){
    sendData=ID + ".txt=" + Value;
    dashSerial->print(sendData);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
}








//bar
void Send_bar(String ID, int Value){
    sendData=ID + ".val=" + String(Value);
    dashSerial->print(sendData);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
}










//positon
void Send_text(String ID, int x, int y){
    sendData=ID + ".x=" + String(x);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    sendData=ID + ".y=" + String(y);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    dashSerial->write(0xff);
    dashSerial->print(sendData);
}








}