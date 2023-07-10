#include <SPI.h>
#include <SD.h>

#include <Adafruit_BMP085.h>

#include "Wire.h"
#include <MPU6050_light.h>

File myFile;
MPU6050 mpu(Wire);
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Initializing SD card... \n"); // SD카드 모듈 초기화 진행SD카드 모듈 초기화 진행
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
  for(int i=1; i<11; i++){
    mpu.update();

    sendExcel 

    myFile = SD.open("Data.txt", FILE_WRITE); 
    if (myFile) { 
      Serial.print("txt파일 작성 중....");
      myFile.println(i);myFile.println("번째 파일");
      myFile.println("Temperature : ");myFile.println(mpu.getTemp());
      
      myFile.println("ACCELERO  X: ");myFile.println(mpu.getAccX());
      myFile.println("ACCELERO  Y: ");myFile.println(mpu.getAccY());
      myFile.println("ACCELERO  Z: ");myFile.println(mpu.getAccZ());

      myFile.println("ACC ANGLE X: ");myFile.println(mpu.getAccAngleX());
      myFile.println("ACC ANGLE Y: ");myFile.println(mpu.getAccAngleY());
    
      myFile.println("ANGLE     X: ");myFile.println(mpu.getAngleX());
      myFile.println("ANGLE     Y: ");myFile.println(mpu.getAngleY());
      myFile.println("ANGLE     Z: ");myFile.println(mpu.getAngleZ());

      myFile.println("GYRO      X: ");myFile.println(mpu.getGyroX());
      myFile.println("GYRO      Y: ");myFile.println(mpu.getGyroY());
      myFile.println("GYRO      Z: ");myFile.println(mpu.getGyroZ());
      
      myFile.close(); 
      Serial.println("완료");
    } else {
      Serial.println("error opening test.txt");
    }
    delay(500);
    if(i==10){
      while(1){}
    }
  }
}
  /* if(millis() - timer > 1000){ // print data every second

    myFile = SD.open("Flight_Data", FILE_WRITE); 
    if (myFile) { 
      Serial.print("test.txt파일 작성 중....");
      myFile.println("Temperature : ");
      myFile.println(mpu.getTemp());
      myFile.close(); 
      Serial.println("완료");
    } else {
      Serial.println("error opening test.txt");
    }
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }
  */
