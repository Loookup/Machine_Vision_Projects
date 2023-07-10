#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10); // RX, Tx ( RX is connected with Buzzer)
char Data = 0; // Data from Bluetooth

int LedNum[] = {2,3,4,5,6,7,8,9,12,13};
int Numset_DPo[10][8] = { // DP on
{1,1,1,1,0,1,1,1}, //9
{1,1,1,1,1,1,1,1}, //8
{1,1,1,0,0,0,0,1}, //7
{1,0,1,1,1,1,1,1}, //6
{1,0,1,1,0,1,1,1}, //5
{0,1,1,0,0,1,1,1}, //4
{1,1,1,1,0,0,1,1}, //3
{1,1,0,1,1,0,1,1}, //2
{0,1,1,0,0,0,0,1}, //1
{1,1,1,1,1,1,0,1} //0
};
int Numset_DPx[10][8] = { // DP off
{1,1,1,1,0,1,1,0}, //9
{1,1,1,1,1,1,1,0}, //8
{1,1,1,0,0,0,0,0}, //7
{1,0,1,1,1,1,1,0}, //6
{1,0,1,1,0,1,1,0}, //5
{0,1,1,0,0,1,1,0}, //4
{1,1,1,1,0,0,1,0}, //3
{1,1,0,1,1,0,1,0}, //2
{0,1,1,0,0,0,0,0}, //1
{1,1,1,1,1,1,0,0} //0
};

unsigned int BUZZ = 10;
unsigned int TONE = 392;

void setup(){
Serial.begin(9600); // Pc - Arduino
mySerial.begin(9600); // Arduino - Bluetooth
for (int i=0; i<10; i++){
pinMode(LedNum[i], OUTPUT);
}
pinMode(BUZZ, OUTPUT); // For BUZZ
}

void loop(){
if (mySerial.available()) { // Bluetooth -> Arduino
Serial.write(Serial.read());
Data = mySerial.read();
if (Data == '1'){
mySerial.println("T-10 Seconds, Ready for Launch");
for(int i=0; i<10; i++){
for(int j=0; j<10; j++){
for(int l=0; l<5; l++){
for(int k=0; k<8; k++){
digitalWrite(LedNum[9],LOW);
digitalWrite(LedNum[8],HIGH);
digitalWrite(LedNum[k],Numset_DPx[j][k]);
}
delay(9);
for(int k=0; k<8; k++){
digitalWrite(LedNum[8],LOW);
digitalWrite(LedNum[9],HIGH);
digitalWrite(LedNum[k],Numset_DPo[i][k]);
}
delay(9);
}
}
Data = mySerial.read();
if (Data == '0'){ // Whenever you want, you can stop System
mySerial.println("Emergency Stop !");
while(1) {
digitalWrite(BUZZ, LOW);
tone(BUZZ, TONE);
delay(500);
tone(BUZZ, TONE);
digitalWrite(BUZZ,HIGH);
noTone(BUZZ);
delay(2000);
}
}
digitalWrite(BUZZ, LOW);
tone(BUZZ, TONE);
delay(100);
digitalWrite(BUZZ,HIGH);
noTone(BUZZ);
}
mySerial.println("We have lift off");
while(1){
for(int i=0; i<10; i++){
digitalWrite(LedNum[8],LOW);
digitalWrite(LedNum[9],LOW);
digitalWrite(LedNum[i],Numset_DPx[10][i]);
digitalWrite(BUZZ, LOW);
tone(BUZZ, TONE);
}
}
}
}
else {
mySerial.println("Waiting for the Launch Signal...");
delay(2500);
}
}
