#include <Arduino.h>
int sensor = A0;
float resolution = 1024.0;
int sample = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("initializing...");
}

float ph(float voltase) {
  return 7 + ((2.5 - voltase)/0.18);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pengukuran = 0;
  for (int i = 0; i < sample; i++)
  {
    /* code */
    pengukuran += analogRead(sensor);
    delay(10);
  }

  float voltase = 5/resolution * pengukuran / sample;
  Serial.print("PH = ");
  Serial.println(ph(voltase));
  delay(3000);
  
}
