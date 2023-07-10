/* Arduino Bluetooth LED Control Practice
 * Arduino UNO, HC-06  
 */

#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 10); // RX, Tx 
char Data = 0;

void setup() {
  Serial.begin(9600); // Pc - Arduino
  mySerial.begin(9600); // Arduino - Bluetooth
}

void loop() {
  if (mySerial.available()) { // Bluetooth -> Arduino
    Serial.write(Serial.read());
    Data = mySerial.read();
    mySerial.print(Data);
    if (Data == '1'){
      mySerial.println("Launch");
    }
    }
  }
  /*
    if (Serial.available()) { // PC -> Arduino
    mySerial.write(Serial.read());
    }
  */
