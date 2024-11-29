#include <SoftwareSerial.h>

/*

Source: 

https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
https://electropeak.com/learn/interfacing-neo-8m-gps-module-with-arduino/
https://docs.arduino.cc/learn/built-in-libraries/software-serial/

NMEO module pins to Arduino pins:
  GND -> GND
  RX -> 3
  TX -> 4
  VCC -> 5V
*/

const byte rxPin = 4;
const byte txPin = 3;
const unsigned long baudRate = 9600;

SoftwareSerial serial(rxPin, txPin);

void setup() {
  Serial.begin(baudRate);
  serial.begin(baudRate);
}

void loop() {
  while (serial.available() > 0){
    byte gpsData = serial.read();
    Serial.write(gpsData);
  }
}
