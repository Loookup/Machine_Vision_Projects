#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SCL 8
#define SDA 9
#define CSB 10
#define SDO 11

Adafruit_BMP280 bmp(CSB, SDA, SDO, SCL);

void setup() {
  Serial.begin(9600);

  if(!bmp.begin()){
    Serial.println(F("NO"));
    while(1){}
  }
}

void loop(){
  delay(100);
  Serial.println("---------------");

  Serial.print(F("Temp : "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
}
