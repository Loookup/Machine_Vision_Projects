#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include <MPU6050_light.h>

File Data;
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Initializing SD card... \n"); 
  if (!SD.begin(4)) { 
    Serial.println("Fail to Initialize \n");
    while (1);
  }
  Serial.println("Suceed ! \n");
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done! \n");
}


void loop() {
  Data = SD.open("TEST02.txt", FILE_WRITE);
  mpu.update();
  
  if (Data) {
    Serial.println("Writing TXT file...");
    Data.println("ACCELERO  X,Y,Z: ");
    Data.println(mpu.getAccX()),Data.println(mpu.getAccY()),Data.println(mpu.getAccZ());
    Data.close();
    Serial.println("Complete !");
    delay(100);
  }
  else {
    Serial.println("Error opening TEST02.txt...");
    delay(100);
  }
}
