#include <SoftwareSerial.h> // For HC-06

SoftwareSerial mySerial(3, 2);  // RX, Tx

char Data = 0;

void setup() {

  mySerial.begin(9600);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(mySerial.available()){
    Serial.println("Serial is Ready \n");
    Data = mySerial.read();

    if(Data == '0'){
      Serial.println("Waiting for Signal... \n");
    }
    else{
      Serial.println("Received Data is : ");
      Serial.print(Data);
      mySerial.print(Data);
      mySerial.println('\n');
    }
  }
  else{
    Serial.println("Waiting for Connection... \n");
  }

  delay(500);

}
