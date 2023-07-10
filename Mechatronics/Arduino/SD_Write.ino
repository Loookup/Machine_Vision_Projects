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

  myFile = SD.open("Data.txt", FILE_WRITE); 
  if (myFile) { 
    Serial.print("test.txt파일 작성 중....");
    myFile.println("Smart Hong / SD Card Test");
    myFile.close(); 
    Serial.println("완료");
  } else {
    Serial.println("error opening test.txt");
  }
}
void loop() {

}
