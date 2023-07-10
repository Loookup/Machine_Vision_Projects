#include <Q2HX711.h>

const byte hx711_data_pin = 2;
const byte hx711_clock_pin = 3;

Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);

void setup() {
  Serial.begin(9600);
}

void loop() {

  double scale = (hx711.read()-8264228);
  Serial.println(scale);
  delay(500);
}
