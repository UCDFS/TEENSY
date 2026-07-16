
/*
  Teensy USB <-> Nextion Serial Bridge

  USB Serial (COM port on PC)
      <
      >
  Hardware Serial1
      <
      >
  Nextion Display

  Connect:
    Teensy TX1 -> Nextion RX
    Teensy RX1 -> Nextion TX
    Teensy GND -> Nextion GND
*/

#define NEXTION_BAUD 115200

void setup()
{
    // USB serial to PC
    Serial.begin(115200);

    // Hardware serial connected to Nextion
    Serial1.begin(NEXTION_BAUD);

    // Give USB a moment to enumerate
    delay(1000);
}

void loop()
{
    // PC -> Nextion
    while (Serial.available())
    {
        Serial1.write(Serial.read());
    }

    // Nextion -> PC
    while (Serial1.available())
    {
        Serial.write(Serial1.read());
    }
}
