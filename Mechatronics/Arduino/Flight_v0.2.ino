#include <SPI.h>
#include <SD.h> // For SD Card Reader
#include "Wire.h" // For MPU 6500
#include <MPU6050_light.h>
#include <SoftwareSerial.h> // For HC-06

SoftwareSerial mySerial(3, 2);  // RX, Tx

File myFile;

MPU6050 mpu(Wire);

char Data = 7;  // Data from Bluetooth, extern

//int num = 0;

int Timer = 20;

int on = 0;



void Ini_SD(){

  Serial.println("Initializing SD card... \n");

  delay(1000);
  
  if (!SD.begin(4)) { 
    
    Serial.println("Fail to Initialize SD card Module\nStop whole System \n");
    
    while(1) {}
    
  }
  else{
    
    Serial.println("Succeed test for sd card reader ! \n");

    delay(500);
    
  }

  myFile = SD.open("Data.txt", FILE_WRITE); 
  
  if (myFile) { 
    
    Serial.print("Openning TXT File... \n");

    delay(500);
     
    Serial.println("Complete ! \n");
    
  } else {
    
    Serial.println("error opening test.txt");

    while(1){}
    
  }

  
};

void Ini_MPU(){

  Wire.begin();
  
  byte status = mpu.begin();
  
  Serial.println("Initializing MPU6500... \n");

  delay(1000);
  
  Serial.println("MPU6050 status: ");
  
  Serial.println(status);
  
  if(status != 0){
    
    Serial.println("Fail to Initialize SD card Module\nStop whole system \n");
    
    while(1) {}
    
  }
  else{
    
    Serial.println("Succeed test for mpu ! \n");
    
    Serial.println("Calculating offsets, do not move MPU6050");
    
    delay(1000);
    
    mpu.calcOffsets(true,true); // gyro and accelero
    
  }

  Serial.println("Whole System is Ready !\n");

  delay(1000);
  
};

void Signal(){

  if(mySerial.available()){
    
    mySerial.write(Serial.read());

    Data = mySerial.read();

  }
  else {
    
    Data = Data;
    
  }

  if(Data == '0'){

      mySerial.println("******Emergency Stop, Whole System is closed****** \n");

      while(1) {}
      
  }

  delay(10);
  
};

void CountDown(){

  Signal();

  if(on == 0){
    
    if(Data == '1' && Timer == 20){

      mySerial.println("T-20 Seconds, Ready for launch \n");

      on++;

      Timer--;

      delay(900);
      
    }
    else {

      mySerial.println("Waiting for Launch Signal... \n");

      delay(2000);
      
    }
    
  }
  else if(on == 1){
    
    if(Timer < 20 && Timer > 5){

      mySerial.print(Timer);

      mySerial.println("\n");

      delay(900);

      Timer--;
      
    }
    else if(Timer < 6 && Timer > 0){

      mySerial.print(Timer);

      mySerial.println("\n");

      delay(400);

      Timer--;
      
    }
    else if(Timer == 0){

      on = 7;
      
    }

  }

  else {

    mySerial.println("Launch System is completed, Rocket is on its way \n");

    delay(500);
    
  }

};


void Data_Save(){

  if(Timer < 5){

    mpu.update();

//    myFile = SD.open("Data.txt", FILE_WRITE); 
//
//    mySerial.println("Checking TXT file...");
      
    //mySerial.print(num);
    //mySerial.println("th ");
  
      if (myFile) { 
      
        mySerial.println("Temperature : ");mySerial.print(mpu.getTemp());
//      
//        mySerial.println("ACCELERO  X: ");mySerial.print(mpu.getAccX());
//        mySerial.println("ACCELERO  Y: ");mySerial.print(mpu.getAccY());
//        mySerial.println("ACCELERO  Z: ");mySerial.print(mpu.getAccZ());
//
//        myFile.println("ACC ANGLE X: ");myFile.println(mpu.getAccAngleX());
//        myFile.println("ACC ANGLE Y: ");myFile.println(mpu.getAccAngleY());
//    
//        myFile.println("ANGLE     X: ");myFile.println(mpu.getAngleX());
//        myFile.println("ANGLE     Y: ");myFile.println(mpu.getAngleY());
//        myFile.println("ANGLE     Z: ");myFile.println(mpu.getAngleZ());
//
//        myFile.println("GYRO      X: ");myFile.println(mpu.getGyroX());
//        myFile.println("GYRO      Y: ");myFile.println(mpu.getGyroY());
//        myFile.println("GYRO      Z: ");myFile.println(mpu.getGyroZ());
      
        myFile.close(); 
      
        mySerial.println("Complete \n");
      
      } 
      else {
        
        mySerial.println("error opening test.txt \n");
        
      }
    
      //num++;

      delay(500);

  }  
  
}


void setup() {

  Serial.begin(9600);
  
  mySerial.begin(9600);

  Ini_SD();

  Ini_MPU();

}

void loop() {

  CountDown();

  Data_Save();

}
