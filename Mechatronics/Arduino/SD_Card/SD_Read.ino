#include <SPI.h>
#include <SD.h>
File myFile;

void setup() {
  Serial.begin(9600);
  Serial.println("SD카드 모듈 초기화 진행");
  if (!SD.begin(4)) { 
    Serial.println("초기화 실패");
    while (1);
  }
  Serial.println("초기화 성공!");

  myFile = SD.open("Data.txt"); 
  if (myFile) { 
    Serial.print("Data.txt 내용 읽기 : ");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    Serial.println("완료");
  } else {
    Serial.println("error opening test.txt");
  }
}

void loop(){
  
}
