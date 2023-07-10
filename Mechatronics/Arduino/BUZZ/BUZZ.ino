int buzzer = 11;

int scales[] = {262,294,330,349,392,440,494,523};

enum{
 _C1 = 0,//도
 _D1,//레
 _E1,//미
 _F1,//파
 _G1,//솔
 _A1,//라
 _B1,//시
 _C2,//도
 _NONE,
};

unsigned char melody[] = {
_G1,_G1,_A1,_A1,_G1,_G1,_E1,_NONE,_G1,_G1,_E1,_E1,_D1,_NONE,
_G1,_G1,_A1,_A1,_G1,_G1,_E1,_NONE,_G1,_E1,_D1,_E1,_C1,_NONE
};

int index = 0;

void setup() {
  pinMode(buzzer,OUTPUT);
}

void loop() {

  if(melody[index] != _NONE)
  {
    tone(buzzer,scales[melody[index]],200);
  }
  index++;
  index %= sizeof(melody);
  delay(500);
}
