/*
 * This Program is aimed for controlling Rocket remotely and gathering data on flight
 * 2 - Generation Rocket
 * 
 * By LOOOK UP. 21.08.22
 * 
 * https://loookup.tistory.com
 * 
 * 
 * 
 * Arduino UNO X2
 * 
 * HC-06 : 5V, RX(2), TX(3) (Android only)
 * SD RE : 5V, CS(4), SCK(13), MOSI(11), MISO(12)
 * MPU6050 : 3.3V, SCL(A5), SDA(A4)
 * LED : R(5), B(6), G(7)
 * BUZZER : +(10)
 * 
 * 
 */



#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h> // For HC-06



SoftwareSerial mySerial(3, 2);  // RX, Tx

File myFile;

MPU6050 mpu(Wire);

volatile char Data = '0';

int T = 10;

unsigned int BUZZ = 10;

unsigned int TONE = 392;

int num = 0;



void Signal() {

  if( T > 0) {

    if(mySerial.available()){
    
      mySerial.write(Serial.read());

      Data = mySerial.read();

      if(Data == 's') {      // Stop Sequence

        mySerial.println("******* Emergency Stop *******\n");
   
        digitalWrite(5, HIGH);

        digitalWrite(BUZZ, LOW);
   
        tone(BUZZ, TONE);

        delay(3000);

        digitalWrite(BUZZ,HIGH);
   
        noTone(BUZZ);

        digitalWrite(5,LOW);

        mySerial.println("Launch System returns to Standby \n"); 

        T = 10;

        Data = '0';

      }

    } else {
    
      mySerial.println("Waiting for Launch Signal... \n");

      Data = Data;
    
    }

  LED(Data);

  } else {

    digitalWrite(7, HIGH);
    
  }

    
  
}

void LED(char Data){

  if(Data == '1' | T < 10) {               // Launch Status

    mySerial.print(T);

    mySerial.println("\n");

    digitalWrite(7, HIGH);

    BUZ();

    delay(700);

    digitalWrite(7, LOW);

    delay(10);

    T--;

//    if(T == 0) {
//
//      mySerial.println("Ignition \n");
//
//      digitalWrite(9,HIGH);
//
//      delay(1000);
//
//      digitalWrite(9,LOW);
//      
//    }
    
  } else {                      // Standby Status

    digitalWrite(6, HIGH);

    delay(500);

    digitalWrite(6, LOW);

    delay(500);

  }
  
}

void Save() {

  mpu.update();

  if(T < 10) {

    myFile = SD.open("Data.txt", FILE_WRITE); 

    if(myFile) {

      Serial.print("txt파일 작성 중....");
      
      myFile.println(num);myFile.println("th");
      
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

      Serial.println("Complete ! \n");
    
    }
    else {

      Serial.println("Fail to open TXT File \n");
    
    }

    num++;

  }
  
}



void BUZ() {

   digitalWrite(BUZZ, LOW);
   
   tone(BUZZ, TONE);
   
   delay(300);
   
   digitalWrite(BUZZ,HIGH);
   
   noTone(BUZZ);

};



void setup() {
  
  Serial.begin(9600);
  
  mySerial.begin(9600);
  
  Wire.begin();
  
  Serial.println("Initializing SD card... \n"); // SD카드 모듈 초기화 진행SD카드 모듈 초기화 진행
  
  if (!SD.begin(4)) { 
    
    Serial.println("Fail to Initialize \n");
    
    while (1) {}
    
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

  pinMode(5,OUTPUT); // R
  pinMode(6,OUTPUT); // B
  pinMode(7,OUTPUT); // G
  pinMode(10,OUTPUT); // BUZZ
  
}


void loop() {

  Signal();

  Save();

  delay(100);
  
}
