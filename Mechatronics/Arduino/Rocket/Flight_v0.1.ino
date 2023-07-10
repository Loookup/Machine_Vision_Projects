#include <SPI.h>
#include <SD.h> // For SD Card Reader
#include "Wire.h" // For MPU 6500
#include <MPU6050_light.h>
#include <SoftwareSerial.h> // For HC-06

SoftwareSerial mySerial(3, 2);  // RX, Tx
File myFile;
MPU6050 mpu(Wire);

char OLD_Data = 0;  // Data from Bluetooth, extern



void Ini_SD(){

  Serial.println("Initializing SD card... \n"); // SD카드 모듈 초기화 진행SD카드 모듈 초기화 진행
  
  if (!SD.begin(4)) { 
    Serial.println("Fail to Initialize SD card Module\nStop whole system \n");
    while (1);
  } 
  else{
    Serial.println("Succeed test for sd card reader ! \n");
  }
 
};

void Ini_MPU(){

  Wire.begin();
  
  byte status = mpu.begin();
  Serial.println("Initializing MPU6500... \n");
  Serial.println("MPU6050 status: ");
  Serial.println(status);
  
  if(status != 0){
    Serial.println("Fail to Initialize SD card Module\nStop whole system \n");
    while(1);
  }
  else{
    Serial.println("Succeed test for mpu ! \n");}
     Serial.println("Calculating offsets, do not move MPU6050");
     delay(1000);
     mpu.calcOffsets(true,true); // gyro and accelero
  }

  Serial.println("Whole System is Ready !\n");
  
};

char Signal() {

  char Data = OLD_Data;

  if(mySerial.available()){
    Serial.write(Serial.read());
    Data = mySerial.read();
  }
  else {
    Data = OLD_Data;
  }
  
  OLD_Data = Data;

  return Data;
  
};

void CountDown(){

  mySerial.println("T-10 Seconds, Ready for launch");
  delay(900);

  for(int i=9; i>0; i--){
    
    char Data = Signal();

    if(Data == '1'){
      mySerial.print(i);
      mySerial.println("\n");
      delay(900);
    }
    else{
      mySerial.println("Emergency Stop, Whole system is closed \n");
      while(1) {}
    }
   
  }

  mySerial.println("Lift OFF ! \n");
  
};



auto Test_SD(){

  myFile = SD.open("Data.txt", FILE_WRITE);

  if(myFile){
    return myFile;
  }
  else{
    
  }
    
  
}

void Check (){

  
}
  


void setup() {
  
  Serial.begin(9600);
  mySerial.begin(9600);
  
  Ini_SD();

  Ini_MPU();

}

void loop() {
  for(int i=1; i<11; i++){
    mpu.update();


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
